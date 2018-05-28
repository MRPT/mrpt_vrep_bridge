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
    bool convert(const float _range[], const int _dataCount, const float& _maxScanDistance,
        const float& _scanningAngle,const CPose3D& _pose,CObservation2DRangeScan& _obj)
    {   _obj.rightToLeft = true;
        _obj.aperture = _scanningAngle;
        _obj.maxRange = _maxScanDistance;
        _obj.sensorPose = _pose;
        const double ang_step = _obj.aperture / (_dataCount - 1);
        const double inv_ang_step = (_dataCount - 1) / _obj.aperture;
        _obj.resizeScan(_dataCount);
        for (std::size_t i = 0; i < _dataCount; i++)
        {
            int j = inv_ang_step * ang_step * i;
            if (j < 0)
                j += _dataCount;
            else
                j -= _dataCount;
            const float r = _range[j];
            _obj.setScanRange(i, r);
            const bool r_valid = ((_obj.scan[i] < (_maxScanDistance * 0.95)) && (_obj.scan[i] > 0));
            _obj.setScanRangeValidity(i, r_valid);
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
    bool convert(const float _position[3], const float _quaternion[4], CPose3D& _pose)
    {
        if((mrpt::square(_quaternion[0])+mrpt::square(_quaternion[1])+mrpt::square(_quaternion[2])+mrpt::square(_quaternion[3]))-1 <1e-3)
        {
            _pose = CPose3D(static_cast<double>(_position[0]),static_cast<double>(_position[1]),static_cast<double>(_position[2]));
        }
        else
        {
            CQuaternionDouble q = mrpt::math::CQuaternionDouble(static_cast<double>(_quaternion[0]),static_cast<double>(_quaternion[1])
                ,static_cast<double>(_quaternion[2]),static_cast<double>(_quaternion[3]));
            _pose = CPose3D(q,static_cast<double>(_position[0]),static_cast<double>(_position[1]),static_cast<double>(_position[2]));
        }
        return true;
    }
    /**
    *This method converts pose,velocity and angular velocity value of an object into a CObservationOdometry class object. It has the following arguments:
    *- <b>_pose3D</b> -> Contains sensor pose.. 
    *- <b>_vel[3]</b> -> Contains x,y,z velocity value of the sensor.
    *- <b>_angularvelocity[3]</b> -> Contains angular velocity across x,y,z axis. 
    */
    bool convert(const CPose3D& _pose3D, const float _vel[3], const float _angularvelocity[3],CObservationOdometry& _obj)
    {
        CPose2D _pose2D = CPose2D(_pose3D);
        _obj.odometry = _pose2D;
        _obj.hasEncodersInfo = false;
        _obj.hasVelocities = true;
        TTwist2D _tTwist = TTwist2D(static_cast<double>(_vel[0]),static_cast<double>(_vel[1]),static_cast<double>(_angularvelocity[2])); // velocity in 2D + angular velocity along z
        _obj.velocityLocal = _tTwist;
        return true;
    }
}
