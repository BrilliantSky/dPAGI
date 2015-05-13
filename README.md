# dPAGI
D Language Interface to RPI's PAGI World
(http://rair.cogsci.rpi.edu/projects/pagi-world/)

Two classes are provided, a Connection for abstracting communication with PAGI World, and an Agent that can perform basic low-level actions. To use dPAGI, Agent should be subclassed.

## Building dPAGI

dPAGI can be build using dub, the D package system (http://code.dlang.org/getting_started)

Execute the following commands after installing dub:

	git clone https://github.com/BrilliantSky/dPAGI.git
	cd dPAGI
	dub build

On Unix/Linux systems, libdpagi.a should be produced. Other software using dPAGI should link to this library.

## Example usage

	import pagi;

	class TaskBot : pagi.Agent
	{
		this(in char[] host, ushort port)
		{
			super(new pagi.Connection(host,port));
		}

		void run()
		{
			// Jump 5 times
			for(uint i=0; i<5; i++)
			{
				// Vertical force
				applyBodyForce(45000, true);

				// Delay
				for(uint j=0; j<10; j++)
					poll();
			}
		}
	}

	void main()
	{
		auto bot = new TaskBot(hostname, port);
		bot.run();
	}
