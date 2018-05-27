#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>  
#include <mrpt/obs/CObservation2DRangeScan.h>
#include "mrpt_bridge/vrep_conversion.h"
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationOdometry.h>


extern "C" {
    #include "extApi.h"
}

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;

namespace vrep_bridge {
    bool convert(const float _range[], const simxFloat& _maxScanDistance,
        const simxFloat& _scanningAngle,const CPose3D& _pose,CObservation2DRangeScan& _obj){
        
        int _dataCount = sizeof(_range)/sizeof(_range[0]);
        _obj.rightToLeft = true;
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
            else if (j >= _dataCount)
                j -= _dataCount;  

            const float r = _range[j];
            _obj.setScanRange(i, r);

            const bool r_valid = ((_obj.scan[i] < (_maxScanDistance * 0.95)) && (_obj.scan[i] > 0));
            _obj.setScanRangeValidity(i, r_valid);
        }
        return true;

    }

    bool convert(const float _position[3], const float _quaternion[4], CPose3D& _pose){
        if((mrpt::square(_quaternion[0])+mrpt::square(_quaternion[1])+mrpt::square(_quaternion[2])+mrpt::square(_quaternion[3]))-1 <1e-3){
            _pose = CPose3D((double)_position[0],(double)_position[1],(double)_position[2]);            
        }
        else{
            CQuaternionDouble q = mrpt::math::CQuaternionDouble((double)_quaternion[0],(double)_quaternion[1]
                ,(double)_quaternion[2],(double)_quaternion[3]);
            _pose = CPose3D(q,(double)_position[0],(double)_position[1],(double)_position[2]);   
        }
        return true;
    }

    bool convert(const CPose3D& _pose3D, const float _vel[3], const float _angularvelocity[3],CObservationOdometry& _obj){
        CPose2D _pose2D = CPose2D(_pose3D);
        _obj.odometry = _pose2D;
        _obj.hasEncodersInfo = false;
        _obj.hasVelocities = true;
        TTwist2D _tTwist = TTwist2D((double)_vel[0],(double)_vel[1],(double)_angularvelocity[2]); // velocity in 2D + angular velocity along z
        _obj.velocityLocal = _tTwist;
        return true;
    }
}
