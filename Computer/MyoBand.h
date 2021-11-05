#ifndef MYOBAND
#define MYOBAND

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <sstream>
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
    ~MyoBand();

    // Returns the gestures/poses from the MyoBand
    myo::Pose getPose();

    // Returns the EMG data from the MyoBand
    std::vector<int8_t> getEMGdata();

    
#ifdef _DEBUG
    // For debugging purposes
    void print();
#endif

private:
    // This is updated in onEmgData()
    std::vector<int8_t> emgSamples;

    // This is set by getPose()
    myo::Pose currentPose;

    // The Hub pointer provides access to the Myo instances from Myo connect
    myo::Hub* pHub = NULL;

    // The Myo pointer provides access to the individual Myo instance
    myo::Myo* pMyo = NULL;

    // This is set by onPair() and onUnpair()
    bool isPaired;

    // This is set by onArmSync() and onArmUnsync().
    bool onArm;
    myo::Arm whichArm;

    // This is set by onUnlocked() and onLocked().
    bool isUnlocked;

public:
    // DeviceListener Callback functions overrides

    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) override;
    void onUnpair(myo::Myo* myo, uint64_t timestamp) override;

    void onUnlock(myo::Myo* myo, uint64_t timestamp) override;
    void onLock(myo::Myo* myo, uint64_t timestamp) override;

    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation, myo::WarmupState warmupState) override;
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp) override;

    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) override;
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg) override;         
};

#endif 
