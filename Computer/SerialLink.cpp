#include "SerialLink.h"
#include <cstring>

SerialLink::SerialLink(std::string& comPort, DWORD baudRate) : comPort(comPort), baudRate(baudRate) {
    Serial = new SimpleSerial((char*)comPort.data(), baudRate);
}

void SerialLink::sendData(std::string testMessage){
    bool is_sent;
    do {
        is_sent = Serial->WriteSerialPort((char*)testMessage.data());
    } while (!is_sent);

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
}




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