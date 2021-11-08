#include "SerialLink.h"

SerialLink::SerialLink(std::string& comPort, DWORD baudRate, Filtering& FilterObject) 
    : comPort(comPort), baudRate(baudRate), pMyoBand(FilterObject.getMyoBandPointer())
{
    Serial = new SimpleSerial((char*)comPort.data(), baudRate);
}

void SerialLink::sendData() {
    // We get our package from the package construtor
    std::string package = packageConstructor();

    // Then we try to send it, and only set isSent to true once the package is actually sent
    while (!isSent)
    {
        // WriteSerialPort return true or false whether or not the package has been written.
        isSent = Serial->WriteSerialPort((char*)package.data());
    }
    // If we were able to send the package, we set isSent to false for the next package
    isSent = false;
}

std::string SerialLink::packageConstructor() {
    std::string EmergencyStop;
    std::string Mode;
    std::string Direction;
    std::string EndByte;

    return EmergencyStop + Mode + Direction + EndByte + EndByte;
}


/*
    
int n = testMessage.length();
 
// declaring character array
char char_array[n + 1];
 
// copying the contents of the
// string to char array
strcpy_s(char_array, testMessage.c_str());
    

bool is_sent;
do {
    is_sent = Serial->WriteSerialPort(char_array);
} while (!is_sent);
*/



/*
#include "SerialLink.h"

char com_port[] = "\\\\.\\COM6";
DWORD COM_BAUD_RATE = CBR_9600;
SerialLink Serial(com_port, COM_BAUD_RATE);

bool EStopIsSent = Serial.WriteSerialPort("test");  
bool ModeIsSent = Serial.WriteSerialPort("test");
bool SignIsSent = Serial.WriteSerialPort("test");
bool SpeedIsSent = Serial.WriteSerialPort("test");
bool EByte1IsSent = Serial.WriteSerialPort("test");
bool EByte2IsSent = Serial.WriteSerialPort("test");

if(connected_) {
    // Read mode and velocity
    // Send data as:
    if (is_sent) {
    //do whatever
    }
}
*/