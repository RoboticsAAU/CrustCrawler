#include "SerialLink.h"

SerialLink::SerialLink(char* comPort, DWORD baudRate, Filtering& FilterObject)
    : comPort(comPort), baudRate(baudRate), pFilterObject(&FilterObject), pMyoBand(FilterObject.getMyoBandPointer()), isSent(false)
{
    // We create our link object, that connects our computer to the arduino
    //Serial = new SimpleSerial(comPort, baudRate);
    Serial = new serialib;

    while (Serial->openDevice(comPort, baudRate) != 1) {
        printf("Serial device could not be found\n");
        Sleep(500);
    }
    std::cout << "Serial device opened\n" << std::endl;

    //We know that our package is 6 byte long, so we reverse that
    package.assign(6, 0);
}

void SerialLink::sendData() {
    // We construct the new package
    packageConstructor();

    //Current time is assigned to the time_stamp used for calibration
    if (calibration == false) {
        if (firstCalibration) {
            startTime = std::chrono::steady_clock::now();
            firstCalibration = false;
        }

        //The user is requested to perform gestures for calibration
        if ((std::chrono::steady_clock::now() - startTime) < std::chrono::seconds::duration(15)) {
            printf("\r");
            printf("Slowly flex your hand right and left for 15 seconds.");
            fflush(stdout);
            return;
        }
        else {
            calibration = true;
            return;
        }
    }

    // Then we try to send it, and only set isSent to true once the package is actually sent. Notice that 1 is subtracted from package.size to avoid sending the terminating character
    for (int i = 0; i < package.size() - 1; i++)
    {
        while (isSent != 1)
        {
            // WriteSerialPort return true or false whether or not the package has been written.
            isSent = Serial->writeChar(package.at(i));
        }
        isSent = 0;
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

void SerialLink::getEmergencyStop(unsigned char& outStop) {
    if (GetKeyState(VK_SPACE)) {
        outStop = 1;
    }
}

void SerialLink::getSpeed(unsigned char& outSpeed) {
    
    if ((previousPose == myo::Pose::waveOut || previousPose == myo::Pose::waveIn) && previousPose != lastControlPose) {
        pFilterObject->resetMoveAvg();
        lastControlPose = previousPose;
    }

    double RAWspeed = pFilterObject->MoveAvg();

    //Check key state for speed control mode
    if (GetKeyState(0x31) & 0x8000) { speedMode = SpeedMode::Gross; }
    else if (GetKeyState(0x32) & 0x8000) { speedMode = SpeedMode::Linear; }
    else if (GetKeyState(0x33) & 0x8000) { speedMode = SpeedMode::Precision; }
    
    //We adjust to user's thresholds
    if (previousPose == myo::Pose::waveOut) {
        if (RAWspeed > waveOutMaxSpeed) {
            waveOutMaxSpeed = RAWspeed;
        }
        if (RAWspeed < waveOutThreshold) {
            waveOutThreshold = RAWspeed;
        }
    }
    if (previousPose == myo::Pose::waveIn) {
        if (RAWspeed > waveInMaxSpeed) {
            waveInMaxSpeed = RAWspeed;
        }
        if (RAWspeed < waveInThreshold) {
            waveInThreshold = RAWspeed;
        }
    }
    
    double adjustedSpeed = 0;

    if (previousPose == myo::Pose::waveOut) {
        adjustedSpeed = RAWspeed - waveOutThreshold;
    }
    else if (previousPose == myo::Pose::waveIn) {
        adjustedSpeed = RAWspeed - waveInThreshold;
    }
    else {
        //Constant deacceleration when neither waveIn or 
        double increment = prevSpeed - 0.2;
        adjustedSpeed = (increment > 0.0) ? increment : 0;
    }
    
    prevSpeed = adjustedSpeed;


    double convertedSpeed = speedMap(adjustedSpeed);

    convertedSpeed = (convertedSpeed > maxSpeedCap) ? maxSpeedCap : convertedSpeed;

    convertedSpeed = (convertedSpeed < 0) ? 0 : convertedSpeed;

    outSpeed = convertedSpeed;
}

void SerialLink::getDirection(unsigned char& outDirection) {
    myo::Pose Pose = pMyoBand->getPose();
    if (Pose == myo::Pose::waveIn) {
        outDirection = 0;
    }
    if (Pose == myo::Pose::waveOut) {
        outDirection = 1;
    }
}

void SerialLink::getMode(unsigned char& outMode) {
    myo::Pose currentPose = pMyoBand->getPose();

    if (currentPose == previousPose || currentPose == myo::Pose::rest || currentPose == myo::Pose::unknown) {
        previousPose = currentPose;
        return;
    }

    switch (controlMode)
    {
    case ControlMode::Grasp: {
        if (currentPose == myo::Pose::fingersSpread) {
            controlMode = ControlMode::LeftRight;
        }
        else if (currentPose == myo::Pose::fist) {
            controlMode = ControlMode::InOut;
        }
        break;
    }
    case ControlMode::LeftRight: {
        if (currentPose == myo::Pose::fingersSpread) {
            controlMode = ControlMode::UpDown;
        }
        else if (currentPose == myo::Pose::fist) {
            controlMode = ControlMode::Grasp;
        }
        break;
    }

    case ControlMode::UpDown: {
        if (currentPose == myo::Pose::fingersSpread) {
            controlMode = ControlMode::InOut;
        }
        else if (currentPose == myo::Pose::fist) {
            controlMode = ControlMode::LeftRight;
        }
        break;
    }

    case ControlMode::InOut: {
        if (currentPose == myo::Pose::fingersSpread) {
            controlMode = ControlMode::Grasp;
        }
        else if (currentPose == myo::Pose::fist) {
            controlMode = ControlMode::UpDown;
        }
        break;
    }

    default: {
        throw std::runtime_error("Invalid Mode");
        break;
    }
    }
    outMode = controlMode;
    previousPose = currentPose;
}

double SerialLink::speedMap(double& variable) {
    double mappedVariable = variable;

    //The maximum moving average is mapped to a "power" ranging from 0 - 100%
    if (previousPose == myo::Pose::waveOut || lastControlPose == myo::Pose::waveOut) {
        mappedVariable = (100 / (waveOutMaxSpeed - waveOutThreshold)) * variable;
    }
    else if (previousPose == myo::Pose::waveIn || lastControlPose == myo::Pose::waveIn){
        mappedVariable = (100 / (waveInMaxSpeed - waveInThreshold)) * variable;
    }

    

    //The power (mappedVariable) is once again mapped to the corresponding cartesian speed in mm/s, according to the selected mode. 
    switch (speedMode) {
    case SpeedMode::Gross: {
        return (74.8 * log10(mappedVariable + 1));
    }
    case SpeedMode::Linear: {
        return (1.5 * mappedVariable);
    }
    case SpeedMode::Precision: {
        return (0.00179091 * pow(mappedVariable, 2.4615091));
    }
    default: {
        throw std::runtime_error("Invalid speedMode");
    }
    }
}


#ifdef _DEBUG
void SerialLink::print() {
    packageConstructor();
    //printf("Mode: %d Sign: %d Speed: %3d EndByte: %d", Mode, Direction, Speed, EndByte);
    printf("HByte: %d EStop: %3d Mode: %d Dir: %3d Speed: %3d CMode: %d SMode: %d",
        HeaderByte, EmergencyStop, Mode, Direction, Speed, controlMode, speedMode);
}
#endif

