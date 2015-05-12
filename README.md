# dPAGI
D Language Interface to RPI's PAGI World
(http://rair.cogsci.rpi.edu/projects/pagi-world/)

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
