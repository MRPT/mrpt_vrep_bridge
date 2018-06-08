#ifndef VREP_CONVERSION_H
#define VREP_CONVERSION_H

#include <cstdint>
#include <cstring>

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
	/**
    * Converts VREP laser scans into a CObservation2DRangeScan class object. It has the following arguments:
    * @param[out] obj Converted CObservation2DRangeScan object.
    * @param[in] _range[] This array contains all the laser scan measurements(distances).
    * @param[in]  _dataCount Contains the size of _range[].
    * @param[in] _maxScanDistance Contains maximum scan distance of scanner(simulation parameter)
    * @param[in] _scanningAngle Contains scanning angle of scanner(simulation parameter)
    * @param[in] _pose Sensor Pose.
    */
	bool convert(const float range[], const int dataCount, const float& maxScanDistance,
		const float& scanningAngle,const mrpt::poses::CPose3D& pose,mrpt::obs::CObservation2DRangeScan& obj);

	/**
    * Converts sensor position,quaternion into a CPose3D class object. It has the following arguments:
    * @param[out] pose Converted CPose3D object.
    * @param[in] position[3] This array contains x,y,z coordinates of the sensor.
    * @param[in] quaternion[4] Contains the quaternion vector of the sensor.
    * <i> Only position array is used for conversion to CPose3D in case _quaternion vector cannot be converted to CQuaternionDouble class<i> 
    */
	bool convert(const float position[3], const float quaternion[4],mrpt::poses::CPose3D& pose);
	
	/**
    * Converts pose,velocity and angular velocity value of an object into a CObservationOdometry class object. It has the following arguments:
    * @param[out] obj Contains the converted CObservationOdometry object.
    * @param[in] _pose3D Contains sensor pose.. 
    * @param[in] _vel[3] Contains x,y,z velocity value of the sensor.
    * @param[in] _angularvelocity[3] Contains angular velocity across x,y,z axis. 
    */
	bool convert(const mrpt::poses::CPose3D& pose3D,const float vel[3], const float angularvelocity[3], mrpt::obs::CObservationOdometry& obj);
}  

#endif  
