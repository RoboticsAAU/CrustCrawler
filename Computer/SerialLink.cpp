#include "SerialLink.h"

SerialLink::SerialLink(char* comPort, DWORD baudRate, Filtering& FilterObject) 
    : comPort(comPort), baudRate(baudRate), pFilterObject(&FilterObject), pMyoBand(FilterObject.getMyoBandPointer()), isSent(false)
{
    // We create our link object, that connects our computer to the arduino
    //Serial = new SimpleSerial(comPort, baudRate);
    Serial = new serialib;
    while (Serial->openDevice(comPort, baudRate) != 1) {
        printf("Serial device could not be found ");
        Sleep(500);
    }
    std::cout << "Serial device opened" << std::endl;

    // We know that our package is 6 byte long, so we reverse that
    package.assign(6, 0);

    std::chrono::time_point<std::chrono::high_resolution_clock> startTime = std::chrono::high_resolution_clock::now();

    std::chrono::time_point<std::chrono::high_resolution_clock> currentTime;

    #ifdef _DEBUG
    printf("Please use your max strength for 5 seconds");
    do{
        int speed = pFilterObject->MoveAvg();
        if ( speed > currentMaxSpeed ) {
            currentMaxSpeed = speed;
        }
        currentTime = std::chrono::high_resolution_clock::now();
    } while (currentTime - startTime < std::chrono::seconds::duration(5));

    startTime = std::chrono::high_resolution_clock::now();

    printf("Please rest your arm for 10 seconds");
    do {
        int speed = pFilterObject->MoveAvg();
        myo::Pose pose = pMyoBand->getPose();

        if ( pose == myo::Pose::rest && speed > threshold) {
            threshold = speed;
        }
        currentTime = std::chrono::high_resolution_clock::now();

    } while (currentTime - startTime < std::chrono::seconds::duration(10));
    #endif

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

    if (previousPose == myo::Pose::rest) {
        outSpeed = (char)0;
        return;
    }
    int convertedSpeed = RAWspeed - threshold;
    convertedSpeed = speedMap(convertedSpeed);
    convertedSpeed = convertedSpeed > speedCap ? speedCap : convertedSpeed;

    outSpeed = (char)(convertedSpeed);
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

int SerialLink::speedMap(int& variable) {
    if (GetKeyState(0x41) & 0x8000) { speedMode = SpeedMode::Gross;  }
    if (GetKeyState(0x53) & 0x8000) { speedMode = SpeedMode::Linear; }
    if (GetKeyState(0x44) & 0x8000) { speedMode = SpeedMode::Precision; }
    
    switch(speedMode){
    case SpeedMode::Gross:{
        return (int) 74.8*log10(variable + 1);
    }
    case SpeedMode::Linear:{
        return (int) 1.5*variable;
    }
    case SpeedMode::Precision:{
        return (int) 1.0918*pow(10,-6)*pow(variable, 4.069);
    }
    default:{
        throw std::runtime_error("Invalid speedMode");
    }
    }
}

#ifdef _DEBUG
void SerialLink::print(){
    packageConstructor();
    //printf("Mode: %d Sign: %d Speed: %3d EndByte: %d", Mode, Direction, Speed, EndByte);
    printf("HeaderByte: %d Emergency Stop: %d Mode: %d Direction: %d Speed: %3d ", 
            package.at(0), package.at(1), package.at(2), package.at(3), package.at(4));
}                                                        
#endif

