#include "SerialLink.h"

SerialLink::SerialLink(char* comPort, DWORD baudRate, Filtering& FilterObject) 
    : comPort(comPort), baudRate(baudRate), pFilterObject(&FilterObject), pMyoBand(FilterObject.getMyoBandPointer()), isSent(false)
{
    // We create our link object, that connects our computer to the arduino
    Serial = new SimpleSerial(comPort, baudRate);

    // We know that our package is 6 byte long, so we reverse that
    package.reserve(6);
}

void SerialLink::sendData() {
    // We construct the new package
    packageConstructor();

    // Then we try to send it, and only set isSent to true once the package is actually sent
    while (!isSent)
    {
        // WriteSerialPort return true or false whether or not the package has been written.
        isSent = Serial->WriteSerialPort((char*)package.c_str());
    }
    // If we were able to send the package, we set isSent to false for the next package
    isSent = false;
}


void SerialLink::packageConstructor() {
    //The current data is fetched
    getEmergencyStop(EmergencyStop);
    getMode(Mode);
    getDirection(Direction);
    getSpeed(Speed);

    //The individual data characters are assigned to the string indices
    package.assign(0, EmergencyStop);
    package.assign(1, Mode);
    package.assign(2, Direction);
    package.assign(3, Speed);
    package.assign(4, EndByte);
    package.assign(5, EndByte);
   
}
void SerialLink::getEmergencyStop(char& outStop){

}

void SerialLink::getSpeed(char& outSpeed) {
    int RAWspeed = pFilterObject->MoveAvg();
    outSpeed = (char)(RAWspeed);
}


void SerialLink::getDirection(char& outDirection){
    myo::Pose Pose = pMyoBand->getPose();
    if(Pose == myo::Pose::waveIn){
        outDirection = 0;
    }
    if(Pose == myo::Pose::waveOut){
        outDirection = 1;
    }
}

void SerialLink::getMode(char& outMode) {
    
    if (std::chrono::system_clock::now() - timeStamp < std::chrono::milliseconds(500)) {
        return;
    }
    
    myo::Pose currentPose = pMyoBand->getPose();

    if (currentPose == previousPose) {
        return;
    }

    timeStamp = std::chrono::system_clock::now();

   
    switch (currentMode)
    {
        case eMode::Grasp: {
            if (currentPose == myo::Pose::fingersSpread) {
                currentMode = eMode::LeftRight;
            }
            else if (currentPose == myo::Pose::fist) {
                currentMode = eMode::InOut;
            }
            break;
        }
        case eMode::LeftRight: {
            if (currentPose == myo::Pose::fingersSpread) {
                currentMode = eMode::UpDown;
            }
            else if (currentPose == myo::Pose::fist) {
                currentMode = eMode::Grasp;
            }
            break;
        }

        case eMode::UpDown: {
            if (currentPose == myo::Pose::fingersSpread) {
                currentMode = eMode::InOut;
            }
            else if (currentPose == myo::Pose::fist) {
                currentMode = eMode::LeftRight;
            }
            break;
        }

        case eMode::InOut: {
            if (currentPose == myo::Pose::fingersSpread) {
                currentMode = eMode::Grasp;
            }
            else if (currentPose == myo::Pose::fist) {
                currentMode = eMode::UpDown;
            }
            break;
        }

        default: {
            throw std::runtime_error("Invalid Mode");
            break;
        }
    }
    outMode = currentMode;
}

#ifdef _DEBUG
void SerialLink::print(){
    packageConstructor();
    printf("Mode: %d Sign: %d Speed: %3d EndByte: %d", Mode, Direction, Speed, EndByte);
}
#endif
