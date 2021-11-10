#include "SerialLink.h"

SerialLink::SerialLink(char* comPort, DWORD baudRate, Filtering& FilterObject) 
    : comPort(comPort), baudRate(baudRate), pFilterObject(&FilterObject), pMyoBand(FilterObject.getMyoBandPointer()), isSent(false)
{
    // We create our link object, that connects our computer to the arduino
    //Serial = new SimpleSerial(comPort, baudRate);
    Serial = new serialib;
    while (Serial->openDevice(comPort, baudRate) != true) {
        printf("Serial device could not be found ");
        Sleep(500);
    }
    std::cout << "Serial device opened" << std::endl;

    // We know that our package is 6 byte long, so we reverse that
    package.assign(6, 0);
}

void SerialLink::sendData() {
    // We construct the new package
    packageConstructor();

    // Then we try to send it, and only set isSent to true once the package is actually sent
    for (int i = 0; i < package.size()-1 ; i++)
    {
        while (isSent != 1)
        {
        // WriteSerialPort return true or false whether or not the package has been written.
            isSent = Serial->writeChar(package.at(i));
        }
        isSent = false;
    }
}


void SerialLink::packageConstructor() {
    //The current data is fetched
    getEmergencyStop(EmergencyStop);
    getMode(Mode);
    getDirection(Direction);
    getSpeed(Speed);

    //The individual data characters are assigned to the string indices
    package.at(0) = HeaderByte;
    package.at(1) = EmergencyStop;
    package.at(2) = Mode;
    package.at(3) = Direction;
    package.at(4) = Speed;
}
void SerialLink::getEmergencyStop(char& outStop){
    if (GetKeyState(VK_SPACE)) {
        outStop = 1;
    }
}

void SerialLink::getSpeed(char& outSpeed) {
    int RAWspeed = pFilterObject->MoveAvg();

    // TODO
    //if (RAWspeed > currentMaxSpeed) { remapProfile(RAWspeed); }
    //int mappedSpeed = speedMap(RAWspeed);

    outSpeed = (char)(mappedSpeed);
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
    myo::Pose currentPose = pMyoBand->getPose();

    if (currentPose == previousPose) {
        return;
    }

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
    previousPose = currentPose;

}





bool SerialLink::aboveThreshold(int& variable) {
    return variable > threshold ? true : false;
}

void SerialLink::gain(int& variable) {

}

#ifdef _DEBUG
void SerialLink::print(){
    packageConstructor();
    //printf("Mode: %d Sign: %d Speed: %3d EndByte: %d", Mode, Direction, Speed, EndByte);
    printf("HeaderByte: %d Emergency Stop: %d Mode: %d Direction: %d Speed: %3d ", 
            package.at(0), package.at(1), package.at(2), package.at(3), package.at(4));
}                                                        
#endif

