#ifndef VREP_CONVERSION_H
#define VREP_CONVERSION_H

#include <stdint.h>
#include <string>

#include <mrpt/version.h>

#include "extApi.h"

namespace mrpt
{
namespace obs
{
class CObservation2DRangeScan;
}
}

namespace mrpt
{
namespace poses
{
class CPose3D;
}
}
namespace vrep_bridge
{ bool convert(const float range[], const simxFloat& _maxScanDistance,
        const simxFloat& _scanningAngle,const mrpt::poses::CPose3D& _pose,mrpt::obs::CObservation2DRangeScan& _obj);
  
  bool convert(const float position[3], const float quaternion[4],mrpt::poses::CPose3D& _pose);
}  

#endif  
