#include "MyoBand.h"

MyoBand::MyoBand() : onArm(true), isUnlocked(false), currentPose() {
        // We heap allocate the Hub object and store a pointer to it
        pHub = new myo::Hub("com.AAU.MyoBand");
        std::cout << "Created Hub" << std::endl;

        // We then get the pointer to the individual Myo instance
        pMyo = pHub->waitForMyo();
        std::cout << "Connected to a Myo armband" << std::endl;

        // We then add our object to the Hub as a Listener, making it possible to retrieve data when a device event occurs
        // A device event is when the MyoBand updates emg data, pose, etc.
        pHub->addListener(this);
}

myo::Pose MyoBand::getPose()
{
    pHub->runOnce(20);
    return myo::Pose();
}

void MyoBand::getEMGdata() {
    pHub->runOnce(20);
}



// Calling pHub->run() or pHub->runOnce() makes the MyoBand call the functions declared in DeviceListener. 
// These functions can be overwritten to do what we like, which is what we will do here below:

void MyoBand::onUnlock(myo::Myo* myo, uint64_t timestamp) {

}

void MyoBand::onLock(myo::Myo* myo, uint64_t timestamp) {

}


#ifdef _DEBUG
void MyoBand::print()
{
    // Clear the current line
    std::cout << '\r';

    if (onArm) {
        // Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

        // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
        // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
        // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
        std::string poseString = currentPose.toString();

        std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
            << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
    }


    // Clear the current line
    //std::cout << '\r';
    // Print out the EMG data.
    for (size_t i = 0; i < emgSamples.size(); i++) {
        std::cout << "[" << static_cast<int>(emgSamples[i]) << "]";
    }
    std::cout << std::flush << std::flush;
}
#endif




/*
void MyoBand::onUnpair(myo::Myo* myo, uint64_t timestamp) {
    onArm = false;
    isUnlocked = false;
}

void MyoBand::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
{
    currentPose = pose;

    if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
        // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
        // Myo becoming locked.
        myo->unlock(myo::Myo::unlockHold);

        // Notify the Myo that the pose has resulted in an action, in this case changing
        // the text on the screen. The Myo will vibrate.
        myo->notifyUserAction();
    }
    else {
        // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
        // are being performed, but lock after inactivity.
        myo->unlock(myo::Myo::unlockTimed);
    }
}

void MyoBand::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
    myo::WarmupState warmupState)
{
    onArm = true;
}

void MyoBand::onArmUnsync(myo::Myo* myo, uint64_t timestamp)
{
    onArm = false;
}

void MyoBand::onUnlock(myo::Myo* myo, uint64_t timestamp)
{
    isUnlocked = true;
}

void MyoBand::onLock(myo::Myo* myo, uint64_t timestamp)
{
    isUnlocked = false;
}

void MyoBand::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg) {
    for (int i = 0; i < 8; i++) {
        emgSamples[i] = emg[i];
    }
}
*/

