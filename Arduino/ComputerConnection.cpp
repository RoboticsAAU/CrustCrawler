#include "ComputerConnection.h"


ComputerConnection::ComputerConnection()
{
	// Initiating serial communication (for data transfer from computer to Arduino and for debug purposes) at given baud rates
	DEBUG_SERIAL.begin(115200);
	DATA_SERIAL.begin(57600);
}

// Function that retrieves the serial data sent from Arduino and assigns it to a "Package" object 
Package ComputerConnection::getPackage()
{
	// If something is available for retrieval on the data serial port
	if (DATA_SERIAL.available()) {
		// The current data on the serial port is assigned to the variable "data"
		int data = (int)DATA_SERIAL.read();
		
		// If the header byte (255) is received, then the four following bytes represent our package
		if (data == 255)
		{   
			// The four bytes are stored into the "databuffer" array and subsequently assigned into their respective package variable
			DATA_SERIAL.readBytes(dataBuffer, 4); // Byte (type of "databuffer") is the same as unsigned char
			returnPackage.EmergencyStop = (bool)dataBuffer[0];
			returnPackage.Mode = (ControlMode)dataBuffer[1];
			returnPackage.Sign = (bool)dataBuffer[2];
			returnPackage.Speed = (uint8_t)dataBuffer[3];
			returnPackage.isUpdated = true;
			return returnPackage;
		}
	}
	// If Arduino has not sent data, the same package is sent (safety measure - should not happen)
	return returnPackage;
}
