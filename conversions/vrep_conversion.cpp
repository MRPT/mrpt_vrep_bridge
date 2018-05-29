#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include "mrpt_bridge/vrep_conversion.h"
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationOdometry.h>

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;

namespace vrep_bridge
{   /**
    *This method converts VREP laser scans into a CObservation2DRangeScan class object. It has the following arguments:
    *- <b>_range[]</b> -> This array contains all the laser scan measurements(distances).
    *- <b>_dataCount</b> -> Contains the size of _range[].
    *- <b>_maxScanDistance,_scanningAngle</b> -> This is a simulation parameter and can be changed in the simulator. Fetched from VREP using simxGetFloatSignal.
    *- <b>_pose</b> -> Sensor Pose.
    */
    bool convert(const float range[], const int dataCount, const float& maxScanDistance,
        const float& scanningAngle,const CPose3D& pose,CObservation2DRangeScan& obj)
    {   obj.rightToLeft = true;
        obj.aperture = scanningAngle;
        obj.maxRange = maxScanDistance;
        obj.sensorPose = pose;
        const double ang_step = obj.aperture / (dataCount - 1);
        const double inv_ang_step = (dataCount - 1) / obj.aperture;
        obj.resizeScan(dataCount);
        for (std::size_t i = 0; i < dataCount; i++)
        {
            int j = inv_ang_step * ang_step * i;
            if (j < 0)
                j += dataCount;
            else
                j -= dataCount;
            const float r = range[j];
            obj.setScanRange(i, r);
            const bool r_valid = ((obj.scan[i] < (maxScanDistance * 0.95)) && (obj.scan[i] > 0));
            obj.setScanRangeValidity(i, r_valid);
        }
        return true;
    }
    /**
    *This method converts sensor position,quaternion into a CPose3D class object. It has the following arguments:
    *- <b>_position[]</b> -> This array contains x,y,z coordinates of the sensor.
    *- <b>_quaternion</b> -> Contains the quaternion vector of the sensor.
    *
    *<i> Only position array is used for conversion to CPose3D in case _quaternion vector cannot be converted to CQuaternionDouble class<i> 
    */
    bool convert(const float position[3], const float quaternion[4], CPose3D& pose)
    {
        if((mrpt::square(quaternion[0])+mrpt::square(quaternion[1])+mrpt::square(quaternion[2])+mrpt::square(quaternion[3]))-1 <1e-3)
        {
            pose = CPose3D(static_cast<double>(position[0]),static_cast<double>(position[1]),static_cast<double>(position[2]));
        }
        else
        {
            CQuaternionDouble q = mrpt::math::CQuaternionDouble(static_cast<double>(quaternion[0]),static_cast<double>(quaternion[1])
                ,static_cast<double>(quaternion[2]),static_cast<double>(quaternion[3]));
            pose = CPose3D(q,static_cast<double>(position[0]),static_cast<double>(position[1]),static_cast<double>(position[2]));
        }
        return true;
    }
    /**
    *This method converts pose,velocity and angular velocity value of an object into a CObservationOdometry class object. It has the following arguments:
    *- <b>_pose3D</b> -> Contains sensor pose.. 
    *- <b>_vel[3]</b> -> Contains x,y,z velocity value of the sensor.
    *- <b>_angularvelocity[3]</b> -> Contains angular velocity across x,y,z axis. 
    */
    bool convert(const CPose3D& pose3D, const float vel[3], const float angularvelocity[3],CObservationOdometry& obj)
    {
        CPose2D pose2D = CPose2D(pose3D);
        obj.odometry = pose2D;
        obj.hasEncodersInfo = false;
        obj.hasVelocities = true;
        TTwist2D tTwist = TTwist2D(static_cast<double>(vel[0]),static_cast<double>(vel[1]),static_cast<double>(angularvelocity[2])); // velocity in 2D + angular velocity along z
        obj.velocityLocal = tTwist;
        return true;
    }
}
