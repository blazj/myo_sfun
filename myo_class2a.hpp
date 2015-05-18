
#ifndef myo_class2a_hpp
#define myo_class2a_hpp
#endif
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <array>
#include <myo.hpp>


// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener 
{
public:
    DataCollector(): onArm(false), isUnlocked(true), currentPose(), emgData(), accData(), gyroData()
    {
    }
    
    ~DataCollector()
    {
    }
    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo.
        // Let's clean up some leftover state.
        roll_e = 0;
        pitch_e = 0;
        yaw_e = 0;
        quat_x, quat_y, quat_z = 0;
        quat_w = 1;
        onArm = false;
        isUnlocked = false;
        emgData.fill(0);
 
    }
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;

        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                        1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
        // Collect angle data
        roll_e = roll;
        pitch_e = pitch;
        yaw_e = yaw;
        // Collect quat data
        quat_x = quat.x();
        quat_y = quat.y();
        quat_z = quat.z();
        quat_w = quat.w();

    }
    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;
        myo->unlock(myo::Myo::unlockHold);
        poseString = currentPose.toString();
    }
    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
    {
        onArm = true;
        whichArm = arm;
        swhichArm=(whichArm == myo::armLeft ? false: true);
    }
    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }
    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = true;
    }
    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = false;
    }
    
    // Get EMG Samples
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t  *emg)
    {
        for (int i = 0; i < 8; i++) {
            emgData[i] = emg[i];
        }
    }    
    void onAccelerometerData (myo::Myo * myo, uint64_t timestamp, const myo::Vector3< float > & accel)
    {
        accData=accel;
    }
    
    void onGyroscopeData (myo::Myo * myo, uint64_t timestamp, const myo::Vector3< float > & gyro )
    {
        gyroData=gyro;
    }
    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.
    // We define this function to print the current values that were updated by the on...() functions above.
   
    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;
    bool swhichArm; 
    
    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;
    // These values are set by onOrientationData() and onPose() above.
    double roll_e, pitch_e, yaw_e;
    double quat_x, quat_y, quat_z, quat_w; 

    myo::Pose currentPose;
    std::string poseString; 
    
    // The values of this array is set by onEmgData() above.
    std::array<int8_t, 8> emgData;
    myo::Vector3<float> accData;
    myo::Vector3<float > gyroData;  
    
};
 


 