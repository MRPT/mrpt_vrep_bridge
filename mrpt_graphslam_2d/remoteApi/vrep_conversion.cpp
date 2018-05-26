#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>  
#include <mrpt/obs/CObservation2DRangeScan.h>
#include "mrpt_bridge/vrep_conversion.h"
#include <mrpt/math/CQuaternion.h>


extern "C" {
    #include "extApi.h"
}

using namespace mrpt::obs;
namespace vrep_bridge {
    bool convert(const float range[], const simxFloat& _maxScanDistance,
        const simxFloat& _scanningAngle,const mrpt::poses::CPose3D& _pose,CObservation2DRangeScan& _obj){
        
        int _dataCount = sizeof(range)/sizeof(range[0]);
        _obj.rightToLeft = true;
        //_obj.sensorLabel 
        _obj.aperture = _scanningAngle;
        _obj.maxRange = _maxScanDistance;
        _obj.sensorPose = _pose; //pose is calculated from the position recieved from the simulation.
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

            const float r = range[j];
            _obj.setScanRange(i, r);

            const bool r_valid = ((_obj.scan[i] < (_maxScanDistance * 0.95)) && (_obj.scan[i] > 0));
            _obj.setScanRangeValidity(i, r_valid);
        }
        return true;

    }

    bool convert(const float position[3], const float quaternion[4], mrpt::poses::CPose3D& _pose){
        mrpt::math::CQuaternionDouble q = mrpt::math::CQuaternionDouble((double)quaternion[0],(double)quaternion[1]
            ,(double)quaternion[2],(double)quaternion[3]);
        _pose = mrpt::poses::CPose3D(q,(double)position[0],(double)position[1],(double)position[2]);
        return true;
    }
}
