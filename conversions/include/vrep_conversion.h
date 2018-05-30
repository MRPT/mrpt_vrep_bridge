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
	bool convert(const float range[], const int dataCount, const float& maxScanDistance,
		const float& scanningAngle,const mrpt::poses::CPose3D& pose,mrpt::obs::CObservation2DRangeScan& obj);
	bool convert(const float position[3], const float quaternion[4],mrpt::poses::CPose3D& pose);
	bool convert(const mrpt::poses::CPose3D& pose3D,const float vel[3], const float angularvelocity[3], mrpt::obs::CObservationOdometry& obj);
}  

#endif  
