#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "mrpt/obs/CObservation2DRangeScan.h"
#include "vrep_conversion.h"
#include "mrpt/math/CQuaternion.h"
#include "mrpt/obs/CObservationOdometry.h"

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;

namespace vrep_bridge
{   
    bool convert(const float range[], const int dataCount, const float& maxScanDistance,
        const float& scanningAngle,const mrpt::poses::CPose3D& pose,mrpt::obs::CObservation2DRangeScan& obj)
    {   obj.rightToLeft = true;
        obj.aperture = scanningAngle;
        obj.maxRange = maxScanDistance;
        obj.sensorPose = pose;
        obj.resizeScan(dataCount);
        for (std::size_t i = 0; i < dataCount; i++)
        {
            const float r = range[i];
            obj.setScanRange(i, r);
            const bool r_valid = ((obj.scan[i] < (maxScanDistance * 0.95)) && (obj.scan[i] > 0));
            obj.setScanRangeValidity(i, r_valid);
        }
        return true;
    }

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
