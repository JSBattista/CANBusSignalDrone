# CAN Bus Signal Drone
Signal Drones for pumping query and response data to throttle control systems for testing during the assembly process.
One of the problems with Electronic (CAN bus) remote control systems is that they are comprised of numerous sub-assemblies that have to be connected to each other during assembly into the whole or greater assembly. 

In that process, there is a lot of room for error. Error that would not be discovered until later. And then, at that point, you have an entire assembly that cannot pass the final tests and it has to be diagnosed, and then entirely taken apart to replace the defective sub-assembly. More often than that, it's usually a loose connector, bent or retracted pin, or something like that. So for one loose connector, not only is the assembly not able to pass the final thorough testing, and hence not shipped, but somebody who would otherwise be building a new assembly is instead having to dismantle one that was already built. 

For this situation the Signal Drone was devised. It's called a drone because all it does is generate a query signal, find out what button is being pressed, and then respond back to the console by flashing that button or some sequence of lights for those buttons and switches that do not have lights. There are some analog signals to deal with as well, which have lights. 

These Signal Drones, or sigdrones, save a lot of time by catching issues before the major assembly is entirely "buttoned up", which is during the assembly process. 

It should be noted that the sigdrones only speak to the test programs running in the console assemblies. On what we call the EOL station, the assemblies get their final program after a thorough testing phase, that will be used out in the field and communicates with the other marine systems they will be connected to. So there are no CAN signals in the sigdrone code that are pertinent to actual boats that these consoles are installed into.

The entire thing is run by an Arduino Nano. It also uses a 2 channel CAN adaptor originally intended for use with Raspberry Pi but it can be used with any MC having SPI connections. It can be found on Amazon for around $27.

The two posted images show the evolution of the design (likely to come about when you keep making them) from first to last, both still in service.  Close observaetion shows in addition to the inexpensive Nano, the cheap buck converter as well, the small screen, and the most advanced unit using a V/A meter too. The most expensive part is the locking metal drawer (to keep people from putting stuff in there) and the Molex connectors. The first drone has been in service since Spring of 2022 without any failures. A 5.2V Zener protects the Nano in case the cheap "by the bag" buck converter from Amazon decides to get frisky. 
