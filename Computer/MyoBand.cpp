#include "MyoBand.h"

MyoBand::MyoBand() : onArm(false), isUnlocked(false), isPaired(true) {
        // We heap allocate the Hub object and store a pointer to it
        pHub = new myo::Hub("com.AAU.MyoBand");
        std::cout << "Created Hub" << std::endl;

        // We then get the pointer to the individual Myo instance
        pMyo = pHub->waitForMyo();
        std::cout << "Connected to a Myo armband" << std::endl;

        // We then add our object to the Hub as a Listener, making it possible to retrieve data when a device event occurs
        // A device event is when the MyoBand updates emg data, pose, etc.
        pHub->addListener(this);

        // We enable the EMG stream
        pMyo->setStreamEmg(myo::Myo::streamEmgEnabled);

        // Since we have 8 channels we reserve 8 elements in emgSamples
        emgSamples.reserve(8);
        emgSamples.insert(emgSamples.begin(), 8, 0);
}

MyoBand::~MyoBand() {
    delete pHub;
}

myo::Pose MyoBand::getPose() {
    if ( !isPaired || !onArm || !isUnlocked ) {
        return myo::Pose::unknown;
    }
    pHub->runOnce(20);
    return currentPose;
}

std::vector<int8_t> MyoBand::getEMGdata() {
    /*
    if ( !isPaired || !onArm || !isUnlocked ) {
        std::vector<int8_t> empty(8, 0);
        return empty;
    }
    */
    pHub->runOnce(20);
    return emgSamples;
}



// Calling pHub->run() or pHub->runOnce() makes the MyoBand call the functions declared in DeviceListener. 
// These functions can be overwritten to do what we like, which is what we will do here below:
void MyoBand::onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
    isPaired = true;
}

void MyoBand::onUnpair(myo::Myo* myo, uint64_t timestamp) {
    isPaired = false;
}

void MyoBand::onUnlock(myo::Myo* myo, uint64_t timestamp) {
    isUnlocked = true;
}

void MyoBand::onLock(myo::Myo* myo, uint64_t timestamp) {
    isUnlocked = false;
}

void MyoBand::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation, myo::WarmupState warmupState) {
    onArm = true;
    whichArm = arm;
}

void MyoBand::onArmUnsync(myo::Myo* myo, uint64_t timestamp) {
    onArm = false;
}

void MyoBand::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {
    currentPose = pose;
}

void MyoBand::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg) {
    //emgSamples.insert(emgSamples.begin(),emg, emg + 8);
    for (int i = 0; i < 8; i++) {
        emgSamples[i] = emg[i];
    }
}

#ifdef _DEBUG
void MyoBand::print()
{
    // We get the current pose and EMGdata
    myo::Pose cPose = getPose();
    std::vector<int8_t> cEMGdata = getEMGdata();

    // We return to the start of the line
    std::cout << '\r';

    std::string poseString = cPose.toString();
    std::cout << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
    
    for (size_t i = 0; i < emgSamples.size(); i++) {
        std::ostringstream oss;
        oss << static_cast<int>(emgSamples[i]);
        std::string emgString = oss.str();
        std::cout << '[' << emgString << std::string(4 - emgString.size(), ' ') << ']';
    }

    
}
#endif

