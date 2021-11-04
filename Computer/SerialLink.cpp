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