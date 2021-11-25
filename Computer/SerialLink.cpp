#include "SerialLink.h"

SerialLink::SerialLink(char* comPort, DWORD baudRate, Filtering& FilterObject)
    : comPort(comPort), baudRate(baudRate), pFilterObject(&FilterObject), pMyoBand(FilterObject.getMyoBandPointer()), isSent(false)
{
    // We create our link object, that connects our computer to the arduino
    Serial = new serialib;

    //A serial connection with the given COM-port and baudRate is attempted untill 1 is returned, meaning successful connection
    while (Serial->openDevice(comPort, baudRate) != 1) {
        printf("Serial device could not be found\n");
        Sleep(500);
    }
    std::cout << "Serial device opened\n" << std::endl;

    //We know that our package is 6 byte long, so we reserve that
    package.assign(6, 0);
}

void SerialLink::sendData() {
    // We construct the new package
    packageConstructor();

//Include guard that is only defined whenever debug mode is not selected. 
#ifdef NDEBUG
    //Current time is assigned to the time_stamp used for calibration
    if (calibration == false) {
        if (firstCalibration) {
            interfaceTimeStamp = std::chrono::steady_clock::now();
            firstCalibration = false;
        }

        //The user is requested to perform gestures for calibration
        if ((std::chrono::steady_clock::now() - interfaceTimeStamp) < std::chrono::seconds::duration(15)) {
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
#endif


    if ((std::chrono::steady_clock::now() - serialDelay) > std::chrono::milliseconds::duration(100)) {
        // Then we try to send it, and only set isSent to true once the package is actually sent. 
        //Notice that 1 is subtracted from package.size to avoid sending the terminating character
        for (int i = 0; i < package.size() - 1; i++)
        {
            while (isSent != 1)
            {
                // WriteSerialPort return true or false whether or not the package has been written.
                isSent = Serial->writeChar(package.at(i));
            }
            isSent = 0;
        }
        serialDelay = std::chrono::steady_clock::now();
    }

}

//Function that determines the contents of the package to be sent and assigns it to the package (vector with unsigned char)
void SerialLink::packageConstructor() {
    //The current data is fetched
    getEmergencyStop();
    getMode();
    getSpeed();  //getSpeed() also defines the direction by calling getDirection()

    //The individual data characters (of type unsigned char) are assigned to the string indices
    package.at(0) = HeaderByte;
    package.at(1) = EmergencyStop;
    package.at(2) = Mode;
    package.at(3) = Direction;
    package.at(4) = Speed;
}

//If space bar is pressed an emergency stop has been initialised, and the EmergencyStop variable is turned true. 
void SerialLink::getEmergencyStop() {
    if (GetKeyState(VK_SPACE)) {
        EmergencyStop = 1;
    }
}

void SerialLink::getSpeed() {
    //The moving average of the 8 EMG channels is assigned to the variable RAWspeed
    double RAWspeed = pFilterObject->MoveAvg(true);

    //Check key state for speed control mode
    if (GetKeyState(0x31) & 0x8000) { speedMode = SpeedMode::Gross; }
    else if (GetKeyState(0x32) & 0x8000) { speedMode = SpeedMode::Linear; }
    else if (GetKeyState(0x33) & 0x8000) { speedMode = SpeedMode::Precision; }
    
    //We adjust to user's thresholds. Both upper and lower thresholds for waveOut and waveIn.
    //Everytime the RAWspeed exceeds the current thresholds (by thresholdTolerance %), they are updated to the current RAWspeed.
    if (currentPose == myo::Pose::waveOut) {
        if (RAWspeed > (1 + thresholdTolerance)*waveOutMaxSpeed) {
            waveOutMaxSpeed = RAWspeed;
        }
        if (RAWspeed < (1 - thresholdTolerance)*waveOutThreshold) {
            waveOutThreshold = RAWspeed;
        }
    }
    if (currentPose == myo::Pose::waveIn) {
        if (RAWspeed > (1 + thresholdTolerance)*waveInMaxSpeed) {
            waveInMaxSpeed = RAWspeed;
        }
        if (RAWspeed < (1 - thresholdTolerance)*waveInThreshold) {
            waveInThreshold = RAWspeed;
        }
    }

    //Variable for the speed after applying thresholds. Assigned in all the directly following if else statements.
    //In the statements, we also always find the current moving direction.
    double adjustedSpeed = 0;

    //If we are not in either waveOut or waveIn, or if we have locked the robot, we decelerate (assign 0's to the moving average array).
    if ((currentPose != myo::Pose::waveOut) && (currentPose != myo::Pose::waveIn) || controlMode == ControlMode::Lock) {
        pFilterObject->Decelerate(true);
        RAWspeed = pFilterObject->MoveAvg(false);
        adjustedSpeed = RAWspeed - Threshold(lastControlPose);

        getDirection(lastControlPose);
    }
    //If we are in waveOut or waveIn.
    else if (currentPose == myo::Pose::waveOut || currentPose == myo::Pose::waveIn) {
        //We see if the current pose is not the same as the last control pose (e.g. if we have gone from waveOut to waveIn immediately).
        //If so, we decelerate to 0, and only then update the lastControlPose to whatever current pose we are in.
        if (currentPose != lastControlPose) {
            pFilterObject->Decelerate(true);
            RAWspeed = pFilterObject->MoveAvg(false);
            adjustedSpeed = RAWspeed - Threshold(lastControlPose);

            if (adjustedSpeed < 1e-5) {
                lastControlPose = currentPose;
            }

            getDirection(lastControlPose);
        }
        //We don't decelerate, meaning instead of assigning 0's in the moving average array, we assing the actual average EMG values. 
        else {
            pFilterObject->Decelerate(false);
            RAWspeed = pFilterObject->MoveAvg(false);
            adjustedSpeed = RAWspeed - Threshold(currentPose);

            getDirection(currentPose);
        }
    }
    

    //We convert the adjustedSpeed (thresholded moving average) in speedMap(). Depending on which speed mode we are in, we get a different output.
    double convertedSpeed = speedMap(adjustedSpeed);

    //Since the adjustedSpeed passed into speedMap() can become negative (if RAWspeed is below threshold), and since tolerance allows the adjustedSpeed to
    //exceed the max permitted speed, we must make sure that the converted speed is within the boundaries (0 - 150). If not, then the max 
    //or min speed is returned, depending on the value of convertedSpeed. 
    convertedSpeed = (convertedSpeed < 0) ? 0 : convertedSpeed;
    convertedSpeed = (convertedSpeed > maxSpeedCap) ? maxSpeedCap : convertedSpeed;

    //Finally, the speed is implicitly casted to unsigned char and assigned to the package variable "Speed"
    Speed = convertedSpeed;
}

//Function that assigns the direction, depending on whether the inputPose is waveIn (0) or waveOut (1).
void SerialLink::getDirection(myo::Pose inputPose) {
    if (inputPose == myo::Pose::waveIn) {
        Direction = 0;
    }
    if (inputPose == myo::Pose::waveOut) {
        Direction = 1;
    }
}

//Function that assgins the mode that we are in.
void SerialLink::getMode() {
    //The current pose is fetched from Myoband
    currentPose = pMyoBand->getPose();

    //If the pose is either the same as the previous pose, is equal to rest or is unknown, there will be no need to update the control mode.
    //The previousPose is thus assigned to be the current and we exit the function
    if (currentPose == previousPose || currentPose == myo::Pose::rest || currentPose == myo::Pose::unknown) {
        previousPose = currentPose;
        return;
    }
    
    //If the user has made a "doubleTap" pose, we go into this statement only once (the first instance of the "doubleTap"). 
    //This is ensured by requiring the previousPose to not be doubleTap. 
    if (currentPose == myo::Pose::doubleTap && previousPose != myo::Pose::doubleTap) {
        if (controlMode != ControlMode::Lock) {
            //PreviousControlMode is a variable that holds the controlMode just before entering lock.
            previousControlMode = controlMode;
            controlMode = ControlMode::Lock;
        }
        else if (controlMode == ControlMode::Lock) {
            //The controlMode is now assigned to the previous mode used just before locking
            controlMode = previousControlMode;
        }
    }
    //If no "doubleTap" pose is registered, we enter a switch that changes between modes,
    //depending on whether fingerspread (increase mode by 1) or fist (decrease mode by 1) is used.
    else {
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

        //If we are in lock, we simply stay there. This case is required so that we don't enter default, if the mode is "Lock".
        case ControlMode::Lock: {
            break;
        }

        default: {
            throw std::runtime_error("Invalid Mode");
            break;
        }
        }
    }
   
    if (controlMode != previousControlMode) {
        pMyoBand->notifyUser();
    }

    //After updating the controlMode above, we assign the mode to the corresponding package variable
    Mode = controlMode;

    //previousPose is updated
    previousPose = currentPose;
}

double SerialLink::speedMap(double& variable) {
    double mappedVariable = variable;

    //The adjusted moving average (function's input parameter) is mapped to a "power percentage" ranging from 0 - 100%, 
    //which makes it correspond to the individual person.
    if (currentPose == myo::Pose::waveOut || lastControlPose == myo::Pose::waveOut) {
        mappedVariable = (100 / (waveOutMaxSpeed - waveOutThreshold)) * variable;
    }
    else if (currentPose == myo::Pose::waveIn || lastControlPose == myo::Pose::waveIn){
        mappedVariable = (100 / (waveInMaxSpeed - waveInThreshold)) * variable;
    }

    //The power percentage (mappedVariable) is once again mapped to the corresponding cartesian speed in mm/s, according to the selected mode. 
    //The functions for 'Gross' and 'Precision' are found using the software "Graph" (and could be changed if found necessary).
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

//Function used to determine what lower threshold to use, depending on the pose. The only input that should be passed is either waveOut or waveIn!
double SerialLink::Threshold(myo::Pose gesture){
    return (gesture == myo::Pose::waveOut) ? waveOutThreshold : waveInThreshold;
}


//Function defined with header guards to print values used for debugging whenever in debug-mode
#ifdef _DEBUG
void SerialLink::print() {
    packageConstructor();
    printf("HByte: %d EStop: %3d Mode: %d Dir: %3d Speed: %3d SMode: %d",
        HeaderByte, EmergencyStop, Mode, Direction, Speed, speedMode);
}
#endif
