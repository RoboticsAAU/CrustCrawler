#include "ComputerConnection.h"

ComputerConnection::ComputerConnection()
{
	DEBUG_SERIAL.begin(57200);
	DATA_SERIAL.begin(57200);
}

Package ComputerConnection::getPackage()
{
	Package returnPackage;
	if (DATA_SERIAL.available()) {
		if (DATA_SERIAL.find(255))
		{
			static unsigned char databuffer[4];
			DATA_SERIAL.readBytes(databuffer, sizeof(databuffer) / sizeof(unsigned char));
			returnPackage.EmergencyStop = (bool)databuffer[0];
			returnPackage.Mode = (uint8_t)databuffer[1];
			returnPackage.Sign = (bool)databuffer[2];
			returnPackage.Speed = (uint8_t)databuffer[3];
			returnPackage.isUpdated = true;
			return returnPackage;
		}
	}
	returnPackage.isUpdated = false;
	return returnPackage;
}
