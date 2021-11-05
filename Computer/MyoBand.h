#ifndef MYOBAND
#define MYOBAND

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <vector>

#include <myo/myo.hpp>

class MyoBand : public myo::DeviceListener
{
public:
    MyoBand();

    // Returns the gestures/poses from the MyoBand
    myo::Pose getPose();

    // Returns the EMG data from the MyoBand
    void getEMGdata();

    
#ifdef _DEBUG
    // For debugging purposes
    void print();
#endif

private:
    // This is updated in onEmgData()
    std::vector<int8_t> emgSamples;

    // This is set by onArmSync() and onArmUnsync().
    bool onArm;

    // This is set by onUnlocked() and onLocked().
    bool isUnlocked;

    // This is set by getPose()
    myo::Pose currentPose;

    // The Hub pointer provides access to the Myo instances from Myo connect
    myo::Hub* pHub = NULL;

    // The Myo pointer provides access to the individual Myo instance
    myo::Myo* pMyo = NULL;


public:
    // DeviceListener Callback functions overrides

    void onUnlock(myo::Myo* myo, uint64_t timestamp) override;
    void onLock(myo::Myo* myo, uint64_t timestamp) override;


         

    /*
    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp);

    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose);

    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation, myo::WarmupState warmupState);

    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp);


    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp);


    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp);

     // onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);
    */
};

#endif 
