
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


class Connection
{
private:
	std.socket.Socket socket;
	char[16384] buffer;
	uint readOffset=0;

public:
	this(in char[] host, ushort port)
	{
		auto addr = new InternetAddress(host,port);
		socket = new std.socket.TcpSocket(addr);
		socket.blocking = true;
		socket.setOption(SocketOptionLevel.SOCKET, SocketOption.RCVTIMEO, dur!"msecs"(100));
	}
	~this()
	{
		socket.close();
	}

	void close()
	{
		socket.shutdown(SocketShutdown.BOTH);
	}

	void send(in char[] msg)
	{
		auto bytes = socket.send(msg);
		if( bytes == std.socket.Socket.ERROR )
			throw new std.socket.SocketOSException("Send() failure");
		assert( bytes == msg.length );
	}
	const(char)[][] receive()
	{
		const(char)[][] lines;

		for(;;)
		{
			// Read data
			//writefln("ReadOffset: %s", readOffset);
			//if( readOffset >= buffer.length )
			//	return lines;
			enforce(readOffset < buffer.length, "Receive buffer not large enough");
			auto bytes = socket.receive(buffer[readOffset..$]);
			//if( bytes == 0 )
			//	return lines;
			if( bytes == socket.ERROR )
			{
				if( wouldHaveBlocked() )
					return lines;
				throw new std.socket.SocketOSException("Receive() failure");
			}
			enforce(bytes > 0, "PAGI World closed the connection");
			//writefln("Rx %s bytes", bytes);
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
					//writefln("Done (%s,%s,%s)", idx, tmpIdx, readOffset);
					// Shift back to start of buffer to make space for more data
					for(int i=0; i<(readOffset-idx); i++)
						buffer[i] = buffer[idx+i];
					readOffset -= idx;

					goto next;
				}

				// Read the line - WARNING: this is mutable!!
				//writefln("Buffer (%s,%s)", idx, tmpIdx);
				lines ~= buffer[idx..tmpIdx].dup;
				//foreach(line; lines)
				//	writefln("Line: '%s'", line);

				idx = tmpIdx;
			} while( true );
		next:
			;
		}
		assert(false);
	}
}

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
	struct Body
	{
		Object self;
		alias self this;
		TactileSensor[8] sensors;
		float velX=0, velY=0;
		float rot=0;
	}
	struct Hand
	{
		Object self;
		alias self this;
		TactileSensor[5] sensors;
	}
protected:
	Connection mConn;
	/+float mPosX=0;
	float mPosY=0;
	float mVelX=0;
	float mVelY=0;
	float mRot=0;+/
	Hand mLeftHand, mRightHand;
	Body mBody;

	SysTime mLastUpdateTime;
	Duration mTimeDelta = Duration.zero;

	enum DetailedVisionDims { width=31, height=21 };
	enum PeripheralVisionDims { width=16, height=11 };
	alias DetailedVisionArray = const(char)[][DetailedVisionDims.width][DetailedVisionDims.height];
	alias PeripheralVisionArray = const(char)[][PeripheralVisionDims.width][PeripheralVisionDims.height];

protected:
	enum TBEvent { LE,LX,RE,RX,BE,BX,RG,RR,LG,LL,OE,OX };
	void triggerBoxEvent(in char[] box, TBEvent event) {};
	void endorphinEvent(float amount, uint location) {};
	void visionUpdateDetailed(in DetailedVisionArray sensors) {};
	void visionUpdatePeripheral(in PeripheralVisionArray sensors) {};
	void foundObject(in char[] object, in char[][] sensors) {};

public:
	this(Connection conn) {
		mConn = conn;
		mLastUpdateTime=Clock.currTime();
		for(uint i=0; i<5; i++)
			poll();
	}

	@property float pos_x() const { return mBody.posX; }
	@property float pos_y() const { return mBody.posY; }
	@property float vel_x() const { return mBody.velX; }
	@property float vel_y() const { return mBody.velY; }
	@property float rotation() const { return mBody.rot; }

// High-level functions

	// Rotate to position
	void resetRotation() { setRotation(0); }
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
	// Stop moving (broken - body velocity is absolute, not relative like forces)
	void stop()
	{
		while( abs(this.vel_x) > 1 )
		{
			applyBodyForce(-this.vel_x*5000, -this.vel_y*5000);
			poll();
		}
	}

// Low-level functions

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
	void applyForce(float force, in char[] code)
	{
		auto cmd = std.string.format("addForce,%s,%s\n", code, cast(int)force*mTimeDelta.total!"msecs"/1000);
		//writefln("send: %s", cmd);
		mConn.send(cmd);
	}

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

	void loadTask(in char[] name)
	{
		mConn.send( std.string.format("loadTask,%s\n", name) );
		// Poll to wait for update?
		for(uint i=0; i<3; i++)
			poll();
	}

	bool poll()
	{
		// Update timer
		auto currTime=Clock.currTime();
		mTimeDelta = currTime - mLastUpdateTime;
		mLastUpdateTime = currTime;



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
			case "BP":
				enforce(tokens.length >= 3, "Too few tokens for BP: "~line);
				mBody.posX = parse!float(tokens[1]);
				mBody.posY = parse!float(tokens[2]);
				break;
			case "S":
				enforce(tokens.length >= 3, "Too few tokens for S: "~line);
				mBody.velX = parse!float(tokens[1]);
				mBody.velY = parse!float(tokens[2]);
				break;
			case "A":
				enforce(tokens.length >= 2, "Too few tokens for A: "~line);
				mBody.rot = parse!float(tokens[1]) % (std.math.PI*2);
				break;
			case "TB":
				enforce(tokens.length >=3, "Too few tokens for TB: "~line);
				triggerBoxEvent( tokens[1], parse!TBEvent(tokens[2]) );
				break;
			case "RD":
				enforce(tokens.length >=3, "Too few tokens for RD: "~line);
				endorphinEvent(parse!float(tokens[1]), parse!uint(tokens[2]));
				break;
			case "L0":
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
				case 'L': sensor = &mLeftHand.sensors[tokens[0][1]-'0']; break;
				case 'R': sensor = &mRightHand.sensors[tokens[0][1]-'0']; break;
				case 'B': sensor = &mBody.sensors[tokens[0][1]-'0']; break;
				}
				sensor.p = (tokens[1] == "1");
				sensor.temp = parse!float(tokens[2]);
				sensor.tx[0] = parse!float(tokens[3]);
				sensor.tx[1] = parse!float(tokens[4]);
				sensor.tx[2] = parse!float(tokens[5]);
				sensor.tx[3] = parse!float(tokens[6]);
				break;
			case "LP":
			case "RP":
				enforce(tokens.length >= 3, "Too few tokens for LP/RP: "~line);
				Hand *sensor = tokens[0][0]=='L' ? &mLeftHand : &mRightHand;
				sensor.posX = parse!float(tokens[1]);
				sensor.posY = parse!float(tokens[2]);
				break;
			case "findObj":
				enforce(tokens.length >= 2, "Too few tokens for findObj: "~line);
				//uint[] locations;
				//locations.length = tokens.length-2;
				//foreach(i, token; tokens[2..$])
				//	locations[i] = to!
				foundObject(tokens[1], tokens[2..$]);
				break;
			case "MDN":
				enforce(tokens.length >= 1+DetailedVisionDims.width*DetailedVisionDims.height);
				DetailedVisionArray sensors;
				for(uint y=0; y<DetailedVisionDims.height; y++)
					for(uint x=0; x<DetailedVisionDims.width; x++)
						sensors[y][x] = tokens[1+y*DetailedVisionDims.width+x];
				visionUpdateDetailed(sensors);
				break;
			case "MPN":
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
			case "":
			default:
				break;
			}
		}
		//writefln("Done poll");
		return true;
	}
}


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


void setRotationTarget(Agent guy, float rotation, float epsilon=10)
{
	guy.setReflex("rotateL", "A|>|"~to!string(rotation+epsilon), "addForce|BR|1");
	guy.setReflex("rotateR", "A|<|"~to!string(rotation-epsilon), "addForce|BR|-1");
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

