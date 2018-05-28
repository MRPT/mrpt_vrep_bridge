#ifndef VREP_CONVERSION_H
#define VREP_CONVERSION_H

#include <stdint.h>
#include <string>

namespace mrpt::obs
{
	class CObservation2DRangeScan;
	class CObservationOdometry;
}

namespace mrpt::poses
{
	class CPose3D;
}

namespace vrep_bridge
{
	bool convert(const float _range[], const int _dataCount, const float& _maxScanDistance,
		const float& _scanningAngle,const mrpt::poses::CPose3D& _pose,mrpt::obs::CObservation2DRangeScan& _obj);
	bool convert(const float _position[3], const float _quaternion[4],mrpt::poses::CPose3D& _pose);
	bool convert(const mrpt::poses::CPose3D& _pose3D,const float _vel[3], const float _angularvelocity[3], mrpt::obs::CObservationOdometry& _obj);
}  

#endif  
