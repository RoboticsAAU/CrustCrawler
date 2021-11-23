#include "ComputerConnection.h"

ComputerConnection::ComputerConnection()
{
	DEBUG_SERIAL.begin(115200);
	DATA_SERIAL.begin(57600);
}

Package ComputerConnection::getPackage()
{
	Package returnPackage;
	if (DATA_SERIAL.available()) {
		int data = (int)DATA_SERIAL.read();
		if (data == 255)
		{
			// Byte is the same as unsigned char
			DATA_SERIAL.readBytes(databuffer, 4);
			returnPackage.EmergencyStop = (bool)databuffer[0];
			returnPackage.Mode = (ControlMode)databuffer[1];
			returnPackage.Sign = (bool)databuffer[2];
			returnPackage.Speed = (uint8_t)databuffer[3];
			returnPackage.isUpdated = true;
			return returnPackage;
		}
	}
	//returnPackage.EmergencyStop = false;
	//returnPackage.Mode = UpDown;
	//returnPackage.Sign = true;
	//returnPackage.Speed = (uint8_t)10;
	returnPackage.isUpdated = false;
	return returnPackage;
}
