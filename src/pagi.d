
import std.socket;
import std.string;
import std.array;
import std.conv;
import std.stdio;
import std.datetime;
import std.algorithm;
import std.math;


// Some math stuff
alias PI = std.math.PI;
float toRadians(float deg)
{
	return deg * PI / 180;
}
float toDegrees(float rad)
{
	return rad * 180 / PI;
}
alias Degree=float;
float angleDiff(Degree a, Degree b)
{
	float diff = a-b;
	if( diff < -180 ) diff += 360;
	if( diff >  180 ) diff -= 360;
	return diff;
}

/// Handles low-level details for communication with PAGI World.
///
/// Handles receiving and transmitting messages. Only complete
/// messages will be returned - message fragmentation is handled
/// correctly in all cases.
///
class Connection
{
private:
	std.socket.Socket socket;
	char[16384] buffer;
	uint readOffset=0;

public:
	/// Construct the connection to the specified host and port
	this(in char[] host, ushort port, uint timeout=100)
	{
		auto addr = new InternetAddress(host,port);
		socket = new std.socket.TcpSocket(addr);
		socket.blocking = true;
		socket.setOption(SocketOptionLevel.SOCKET, SocketOption.RCVTIMEO, dur!"msecs"(timeout));
	}
	~this()
	{
		socket.close();
	}

	/// Close the socket (will result in inability to receive or transmit)
	void close()
	{
		socket.shutdown(SocketShutdown.BOTH);
	}

	/// Send a message to PAGI World
	///
	/// Does not append the required newline.
	void send(in char[] msg)
	{
		auto bytes = socket.send(msg);
		if( bytes == std.socket.Socket.ERROR )
			throw new std.socket.SocketOSException("Send() failure");
		assert( bytes == msg.length );
	}
	/// Receive messages from PAGI World
	///
	/// Separate messages are returned in an array and will never be fragmented. They will not contain a trailing newline.
	const(char)[][] receive()
	{
		const(char)[][] lines;

		for(;;)
		{
			// Read data
			enforce(readOffset < buffer.length, "Receive buffer not large enough");
			auto bytes = socket.receive(buffer[readOffset..$]);
			if( bytes == socket.ERROR )
			{
				if( wouldHaveBlocked() )
					return lines;
				throw new std.socket.SocketOSException("Receive() failure");
			}
			enforce(bytes > 0, "PAGI World closed the connection");
			readOffset += bytes;

			// Parse data
			ptrdiff_t idx=0, tmpIdx=0;
			do
			{
				// Skip newlines
				while( buffer[idx] == '\n' )
				{
					idx++;
					if( idx>=readOffset )
					{
						readOffset = 0;
						goto next;
					}
				}

				tmpIdx = std.string.indexOf(buffer[0..readOffset], '\n', idx);
				if( tmpIdx<0 ) // No newline - partial response read
				{
					// Shift back to start of buffer to make space for more data
					for(int i=0; i<(readOffset-idx); i++)
						buffer[i] = buffer[idx+i];
					readOffset -= idx;

					goto next;
				}

				// Read the line - WARNING: this is mutable!!
				lines ~= buffer[idx..tmpIdx].dup;

				idx = tmpIdx;
			} while( true );
		next:
			;
		}
		assert(false);
	}
}

/// Represents PAGI Guy.
///
/// This should be subclassed to provide specialized funcionality
/// for the given task.
class Agent
{
public:
	struct TactileSensor
	{
		bool p;
		float temp;
		float[4] tx;
	}
	struct Object
	{
		float posX=0, posY=0;
	}
	/// PAGI Guy's body
	struct Body
	{
		Object self;
		alias self this;
		TactileSensor[8] sensors;
		float velX=0, velY=0;
		float rot=0;
	}
	/// PAGI Guy's left or right hand
	struct Hand
	{
		Object self;
		alias self this;
		TactileSensor[5] sensors;
	}
protected:
	Connection mConn;           /// Connection to PAGI World
	Hand mLeftHand, mRightHand; /// Hand sensors
	Body mBody;                 /// Body sensors

	/// Used for timing update speeds to adjust forces. Longer delays
	/// result in stronger forces to help keep constant velocity.
	SysTime mLastUpdateTime;
	/// Ditto
	Duration mTimeDelta = Duration.zero;

	enum DetailedVisionDims { width=31, height=21 }; /// Vision sizes
	enum PeripheralVisionDims { width=16, height=11 }; /// Ditto
	alias DetailedVisionArray = const(char)[][DetailedVisionDims.width][DetailedVisionDims.height]; /// Detailed vision array
	alias PeripheralVisionArray = const(char)[][PeripheralVisionDims.width][PeripheralVisionDims.height]; /// Peripheral vision array

protected:
	enum TBEvent { LE,LX,RE,RX,BE,BX,RG,RR,LG,LL,OE,OX };  /// Trigger box event types
	void triggerBoxEvent(in char[] box, TBEvent event) {}; /// Callback for trigger boxes
	void endorphinEvent(float amount, uint location) {};   /// Callback for endorphins
	void visionUpdateDetailed(in DetailedVisionArray sensors) {};     /// Callback for vision updates
	void visionUpdatePeripheral(in PeripheralVisionArray sensors) {}; /// Callback for vision updates
	void foundObject(in char[] object, in char[][] sensors) {};       /// Callback for object detection

public:
	this(Connection conn) {
		mConn = conn;
		mLastUpdateTime=Clock.currTime();
		for(uint i=0; i<5; i++)
			poll();
	}

	// Positional information
	@property float pos_x() const { return mBody.posX; }
	@property float pos_y() const { return mBody.posY; }
	@property float vel_x() const { return mBody.velX; }
	@property float vel_y() const { return mBody.velY; }
	@property float rotation() const { return mBody.rot; }

// High-level functions

	/// Rotate to 0 degrees
	void resetRotation() { setRotation(0); }
	/// Rotate to position
	void setRotation(Degree rot, Degree ep=4)
	{
		float diff = angleDiff(rot, toDegrees(this.rotation));
		writefln("Rotation: %s / %s (%s)", toDegrees(this.rotation), rot, diff);
		while( abs(diff) > ep )
		{
			applyTorque( diff*2 );// diff > 0 ? 40 : -40 );
			poll();
			diff = angleDiff(rot, toDegrees(this.rotation));
			writefln("Rotation: %s / %s (%s)", toDegrees(this.rotation), rot, diff);
		}
	}
	/// Stop moving (broken - body velocity is absolute, not relative like forces)
	void stop()
	{
		while( abs(this.vel_x) > 1 )
		{
			applyBodyForce(-this.vel_x*5000, -this.vel_y*5000);
			poll();
		}
	}

// Low-level functions

// Movement

	void applyTorque(float torque)
	{
		applyForce(torque, "BR");
	}
	void applyBodyForce(float horiz, float vert)
	{
		applyBodyForce(horiz, false);
		applyBodyForce(vert, true);
	}
	void applyBodyForce(float force, bool vertical=false)
	{
		auto code = vertical ? "BMV" : "BMH";
		applyForce(force, code);
	}
	void applyHandForce(float force, bool left, bool vertical)
	{
		string code;
		if( left )
			code = vertical ? "LHV" : "LHH";
		else
			code = vertical ? "RHV" : "RHH";
		applyForce(force, code);
	}
	void applyHandForce(float horiz, float vert, bool left)
	{
		if( left ) {
			applyForce(horiz, "LHH");
			applyForce(vert, "LHV");
		} else {
			applyForce(horiz, "RHH");
			applyForce(vert, "RHV");
		}
	}

// Gripping
	void releaseGrip(bool left) { setGrip(left,false); }
	void setGrip(bool left, bool enable=true)
	{
		string code;
		if( left )
			code = enable ? "LHG" : "LHR";
		else
			code = enable ? "RHG" : "RHR";
		applyForce(1, code);
	}

// Apply generalized force
	void applyForce(float force, in char[] code)
	{
		auto cmd = std.string.format("addForce,%s,%s\n", code, cast(int)force*mTimeDelta.total!"msecs"/1000);
		//writefln("send: %s", cmd);
		mConn.send(cmd);
	}

// Sensor requests - send when needed before poll() command

	void requestBodySensorUpdate(uint n) { requestTactileSensorUpdate('B', n); }
	void requestHandSensorUpdate(bool left, uint n) { requestTactileSensorUpdate(left ? 'L' : 'R', n); }
	void requestTactileSensorUpdate(char code, uint n)
	{
		mConn.send(std.string.format("sensorRequest,%s%s\n", code, n));
	}
	void requestHandPosUpdate(bool left)
	{
		mConn.send(std.string.format("sensorRequest,%s\n", left?"LP":"RP"));
	}
	void requestPeripheralVisionUpdate()
	{
		mConn.send("sensorRequest,MPN\n");
	}
	void requestDetailedVisionUpdate()
	{
		mConn.send("sensorRequest,MDN\n");
	}
	enum SearchMode : string { Peripheral="P", Detailed="D", Both="PD" };
	void findObject(in char[] object, SearchMode mode)
	{
		auto cmd = std.string.format("findObj,%s,%s\n", object, cast(char[])mode);
		mConn.send(cmd);
	}

// States & Reflexes (EXPERIMENTAL!)
	void setReflex(in char[] name, in char[] cond, in char[] action)
	{
		auto actionstr = action;//join(";",actions);
		auto cmd = std.string.format("setReflex,%s,%s,%s\n", name, cond, actionstr);
		writefln("Set reflex: %s", cmd);
		mConn.send(cmd);
	}
	void removeReflex(in char[] name)
	{
		mConn.send( std.string.format("removeReflex,%s\n", name) );
	}
	void setState(in char[] state, int msActive=-1)
	{
		auto cmd = std.string.format("setState,%s,%s\n", state, msActive);
		mConn.send(cmd);
	}
	void removeState(in char[] name)
	{
		mConn.send( std.string.format("removeState,%s,0\n", name) );
	}

// misc.

	/// Load a task in PAGI World.
	void loadTask(in char[] name)
	{
		mConn.send( std.string.format("loadTask,%s\n", name) );
		// Poll to wait for update?
		for(uint i=0; i<3; i++)
			poll();
	}

	/// Poll the connection for new data.
	///
	/// Returns false if no data is available.
	bool poll()
	{
		// Update timer
		auto currTime=Clock.currTime();
		mTimeDelta = currTime - mLastUpdateTime;
		mLastUpdateTime = currTime;


		// Always update body position, rotation, and velocity
		mConn.send("sensorRequest,BP\n");
		mConn.send("sensorRequest,S\n");
		mConn.send("sensorRequest,A\n");

		auto responses = mConn.receive();
		//foreach( line; responses )
		//	writefln("=====: '%s'", line);
		if( responses.empty() )
			return false;
		//writefln("# Responses: %s", responses.length);
		foreach( line; responses )
		{
			//writefln("Response: '%s'", line);
			//continue;
			auto tokens = line.split(",");
			if( tokens.empty )
				continue;

			switch(tokens[0])
			{
			case "BP": // Body Position
				enforce(tokens.length >= 3, "Too few tokens for BP: "~line);
				mBody.posX = parse!float(tokens[1]);
				mBody.posY = parse!float(tokens[2]);
				break;
			case "S": // Body velocity
				enforce(tokens.length >= 3, "Too few tokens for S: "~line);
				mBody.velX = parse!float(tokens[1]);
				mBody.velY = parse!float(tokens[2]);
				break;
			case "A": // Body Rotation
				enforce(tokens.length >= 2, "Too few tokens for A: "~line);
				mBody.rot = parse!float(tokens[1]) % (std.math.PI*2);
				break;
			case "TB": // Trigger Box
				enforce(tokens.length >=3, "Too few tokens for TB: "~line);
				triggerBoxEvent( tokens[1], parse!TBEvent(tokens[2]) );
				break;
			case "RD": // Endorphins
				enforce(tokens.length >=3, "Too few tokens for RD: "~line);
				endorphinEvent(parse!float(tokens[1]), parse!uint(tokens[2]));
				break;
			case "L0": // Hand and body positions
			case "L1":
			case "L2":
			case "L3":
			case "L4":
			case "R0":
			case "R1":
			case "R2":
			case "R3":
			case "R4":
			case "B0":
			case "B1":
			case "B2":
			case "B3":
			case "B4":
			case "B5":
			case "B6":
			case "B7":
				enforce(tokens.length >= 7, "Too few tokens for Bn/Ln/Rn: "~line);
				TactileSensor *sensor;
				final switch( tokens[0][0] )
				{
				case 'L': sensor = &mLeftHand.sensors[tokens[0][1]-'0']; break; // Left hand
				case 'R': sensor = &mRightHand.sensors[tokens[0][1]-'0']; break; // Right hand
				case 'B': sensor = &mBody.sensors[tokens[0][1]-'0']; break; // Body
				}
				sensor.p = (tokens[1] == "1");
				sensor.temp = parse!float(tokens[2]);
				sensor.tx[0] = parse!float(tokens[3]);
				sensor.tx[1] = parse!float(tokens[4]);
				sensor.tx[2] = parse!float(tokens[5]);
				sensor.tx[3] = parse!float(tokens[6]);
				break;
			case "LP": // Left and right hand proprioception
			case "RP":
				enforce(tokens.length >= 3, "Too few tokens for LP/RP: "~line);
				Hand *sensor = tokens[0][0]=='L' ? &mLeftHand : &mRightHand;
				sensor.posX = parse!float(tokens[1]);
				sensor.posY = parse!float(tokens[2]);
				break;
			case "findObj": // Find object
				enforce(tokens.length >= 2, "Too few tokens for findObj: "~line);
				//uint[] locations;
				//locations.length = tokens.length-2;
				//foreach(i, token; tokens[2..$])
				//	locations[i] = to!
				foundObject(tokens[1], tokens[2..$]);
				break;
			case "MDN": // Detailed vision update
				enforce(tokens.length >= 1+DetailedVisionDims.width*DetailedVisionDims.height);
				DetailedVisionArray sensors;
				for(uint y=0; y<DetailedVisionDims.height; y++)
					for(uint x=0; x<DetailedVisionDims.width; x++)
						sensors[y][x] = tokens[1+y*DetailedVisionDims.width+x];
				visionUpdateDetailed(sensors);
				break;
			case "MPN": // Peripheral vision update
				enforce(tokens.length >= 1+PeripheralVisionDims.width*PeripheralVisionDims.height);
				PeripheralVisionArray sensors;
				for(uint y=0; y<PeripheralVisionDims.height; y++)
					for(uint x=0; x<PeripheralVisionDims.width; x++)
					{
						//writefln("x,y: %s,%s / %s,%s; %s", x,y, sensors.length, sensors[y].length, tokens.length);
						sensors[y][x] = tokens[1+y*PeripheralVisionDims.width+x];
					}
				visionUpdatePeripheral(sensors);
				break;
			case "": // Nothing?
			default: // Unknown
				break;
			}
		}
		//writefln("Done poll");
		return true;
	}
}

version(unittest)
{

class TestAgent : Agent
{
	this(Connection conn) { super(conn); }

	override void triggerBoxEvent(in char[] box, TBEvent event)
	{
		writefln("Trigger: %s, %s", box, to!string(event));
	}
	override void endorphinEvent(float amount, uint location)
	{
		writefln("Endorphins: %s @ %s", amount, location);
	}
	override void foundObject(in char[] object, in char[][] sensors)
	{
		writefln("Found object %s:", object);
		foreach( s; sensors )
			writefln("\tSensor %s", s);
	}
	override void visionUpdateDetailed(in DetailedVisionArray sensors)
	{
		writeln("Vision update (Detailed):");
		foreach( i,s; sensors )
			writefln("\tSensor %s: %s", i,s);
	}
	override void visionUpdatePeripheral(in PeripheralVisionArray sensors)
	{
		writeln("Vision update (Peripheral):");
		foreach( i,s; sensors )
			writefln("\tSensor %s: %s", i,s);
	}
}

/+
void setRotationTarget(Agent guy, float rotation, float epsilon=10)
{
	guy.setReflex("rotateL", "A|>|"~to!string(rotation+epsilon), "addForce|BR|1");
	guy.setReflex("rotateR", "A|<|"~to!string(rotation-epsilon), "addForce|BR|-1");
}
+/
}

/+


class State
{
protected:
	void enter() {}
	void exit() {}
	void execute() {}
}

class StateManager
{
private:
	bool[State] mStates;
public:
	

}

+/
/+
void main()
{
	auto guy = new TestAgent( new Connection("192.168.0.15", 42209) );
	writeln("Connected!");

	for(;;)
	{
		if( guy.poll() )
		{
			writefln("Position: %s,%s", guy.pos_x, guy.pos_y);
			//writefln("Rotation: %s", guy.rotation);
			guy.applyBodyForce(-guy.pos_x*1000, false);
			guy.applyTorque(-guy.rotation*100);
			//guy.findObject("poison", guy.SearchMode.Peripheral);
			guy.requestDetailedVisionUpdate();
		}
		else
			writeln(">>>>> No response! <<<<<<<<<<<<<<<");
	}
}
+/

