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
class CObservationOdometry;
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
{ bool convert(const float _range[], const simxFloat& _maxScanDistance,
        const simxFloat& _scanningAngle,const mrpt::poses::CPose3D& _pose,mrpt::obs::CObservation2DRangeScan& _obj);
  
  bool convert(const float _position[3], const float _quaternion[4],mrpt::poses::CPose3D& _pose);

  bool convert(const mrpt::poses::CPose3D& _pose3D,const float _vel[3], const float _angularvelocity[3], mrpt::obs::CObservationOdometry& _obj);
}  

#endif  
