#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include "mrpt/obs/CObservation2DRangeScan.h"
#include "vrep_conversion.h"
#include "mrpt/math/CQuaternion.h"
#include "mrpt/obs/CObservationOdometry.h"

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;

namespace vrep_bridge
{   /**
    *Converts VREP laser scans into a CObservation2DRangeScan class object. It has the following arguments:
    *@param[out] obj Converted CObservation2DRangeScan object.
    *@param[in] _range[] This array contains all the laser scan measurements(distances).
    *@param[in]  _dataCount Contains the size of _range[].
    *@param[in] _maxScanDistance Contains maximum scan distance of scanner(simulation parameter)
    *@param[in] _scanningAngle Contains scanning angle of scanner(simulation parameter)
    *@param[in] _pose Sensor Pose.
    */
    bool convert(const float range[], const int dataCount, const float& maxScanDistance,
        const float& scanningAngle,const mrpt::poses::CPose3D& pose,mrpt::obs::CObservation2DRangeScan& obj)
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
    *Converts sensor position,quaternion into a CPose3D class object. It has the following arguments:
    *@param[out] pose Converted CPose3D object.
    *@param[in] position[3] This array contains x,y,z coordinates of the sensor.
    *@param[in] quaternion[4] Contains the quaternion vector of the sensor.
    *<i> Only position array is used for conversion to CPose3D in case _quaternion vector cannot be converted to CQuaternionDouble class<i> 
    */
    bool convert(const float position[3], const float quaternion[4], mrpt::poses::CPose3D& pose)
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
    *Converts pose,velocity and angular velocity value of an object into a CObservationOdometry class object. It has the following arguments:
    *@param[out] obj Contains the converted CObservationOdometry object.
    *@param[in] _pose3D Contains sensor pose.. 
    *@param[in] _vel[3] Contains x,y,z velocity value of the sensor.
    *@param[in] _angularvelocity[3] Contains angular velocity across x,y,z axis. 
    */
    bool convert(const mrpt::poses::CPose3D& pose3D, const float vel[3], const float angularvelocity[3],mrpt::obs::CObservationOdometry& obj)
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
