#include <gtest/gtest.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include "mrpt/obs/CObservation2DRangeScan.h"
#include "mrpt/obs/CObservationOdometry.h"
#include "mrpt/poses/CPose3D.h"
#include "vrep_conversion.h"
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace vrep_bridge;

/**
*This unit test tests the CObservation2DRangeScan conversion method. It tests the following parameters of the object created with the functions parameters.
*- <b>obj.aperture[]</b> -> scanningAngle
*- <b>obj.maxRange</b> -> maxScanDistance
*- <b>obj.sensorPose</b> -> sensor_pose
*
*/
TEST(ConvertTest, CObservation2DRangeScanTest)
{
	float position[3] ={0.175f,0.00f,0.035f} ,scanningAngle = 4.1867f ,maxScanDistance = 5.00f;
	float range[] = 
	{
		0.522596f ,0.520694f ,0.5189f ,0.516002f ,0.515378f ,0.514868f ,0.512759f ,0.513314f ,0.511191f ,0.512639f ,
		0.51111f ,0.512957f ,0.512058f ,0.515077f ,0.514384f ,0.514165f ,0.518676f ,0.518822f ,0.519768f ,0.52625f ,
		0.528463f ,0.530869f ,0.534459f ,0.552093f ,0.514051f ,0.51159f ,0.50079f ,0.494321f ,0.490789f ,0.488556f ,
		0.485548f ,0.483604f ,0.481487f ,0.480994f ,0.480249f ,0.478227f ,0.478654f ,0.478947f ,0.477009f ,0.478284f ,
		0.479595f ,0.477731f ,0.479953f ,0.482361f ,0.480562f ,0.484045f ,0.482289f ,0.486024f ,0.491505f ,0.489796f ,
		0.496636f ,0.494958f ,0.50294f ,0.523664f ,0.521971f ,0.463143f ,0.462066f ,0.44504f ,0.444043f ,0.436001f ,
		0.435061f ,0.429915f ,0.429025f ,0.425276f ,0.424431f ,0.422605f ,0.4218f ,0.420262f ,0.419496f ,0.418933f ,
		0.418204f ,0.418977f ,0.418282f ,0.418851f ,0.41819f ,0.417546f ,0.419639f ,0.419027f ,0.421455f ,0.420873f ,
		0.423868f ,0.423317f ,0.427758f ,0.427235f ,0.432961f ,0.432465f ,0.439457f ,0.438988f ,0.45503f ,0.45458f ,
		0.411491f ,0.41146f ,0.400607f ,0.400607f ,0.400622f ,0.393857f ,0.393901f ,0.38893f ,0.389004f ,0.385895f ,
		0.385997f ,0.383287f ,0.383418f ,0.38166f ,0.381818f ,0.381992f ,0.380751f ,0.380953f ,0.380026f ,0.380256f ,
		0.38067f ,0.38093f ,0.38204f ,0.38233f ,0.383886f ,0.384206f ,0.384542f ,0.387214f ,0.387583f ,0.39055f ,
		0.390952f ,0.39616f ,0.396598f ,0.40411f ,0.404589f ,0.418109f ,0.418637f ,0.385371f ,0.386406f ,0.375865f ,
		0.376908f ,0.370591f ,0.371654f ,0.366174f ,0.361973f ,0.363061f ,0.359515f ,0.360629f ,0.357786f ,0.35893f ,
		0.356348f ,0.354518f ,0.355702f ,0.353672f ,0.354889f ,0.35391f ,0.355163f ,0.354233f ,0.353274f ,0.354579f ,
		0.354433f ,0.354312f ,0.355676f ,0.355701f ,0.357108f ,0.357816f ,0.358547f ,0.360023f ,0.361401f ,0.363093f ,
		0.364649f ,0.367767f ,0.370789f ,0.372441f ,0.377908f ,0.386399f ,0.390203f ,0.383448f ,0.378072f ,0.374658f ,
		0.371442f ,0.369523f ,0.36294f ,0.361983f ,0.361103f ,0.360299f ,0.359571f ,0.355979f ,0.355883f ,0.355851f ,
		0.35292f ,0.353389f ,0.353917f ,0.351983f ,0.352645f ,0.351138f ,0.352226f ,0.351245f ,0.35246f ,0.351627f ,
		0.353249f ,0.353141f ,0.353116f ,0.354935f ,0.355285f ,0.356074f ,0.358392f ,0.359699f ,0.361776f ,0.364492f ,
		0.364054f ,0.367074f ,0.371484f ,0.371965f ,0.377694f ,0.379585f ,0.39053f ,0.406653f ,0.406642f ,0.39847f ,
		0.393974f ,0.389529f ,0.386683f ,0.383892f ,0.382122f ,0.380322f ,0.378562f ,0.377526f ,0.376507f ,0.375526f ,
		0.373195f ,0.372283f ,0.372216f ,0.371937f ,0.371696f ,0.371887f ,0.372259f ,0.372676f ,0.370578f ,0.371062f ,
		0.372221f ,0.373365f ,0.374565f ,0.372583f ,0.374369f ,0.376379f ,0.378465f ,0.381356f ,0.379459f ,0.382617f ,
		0.385892f ,0.384044f ,0.389294f ,0.395425f ,0.403015f ,0.401181f ,0.416441f ,0.359556f ,0.358512f ,0.347937f ,
		0.346959f ,0.340757f ,0.339829f ,0.336336f ,0.335451f ,0.332241f ,0.331396f ,0.329413f ,0.328604f ,0.326725f ,
		0.325805f ,0.325047f ,0.324599f ,0.323872f ,0.323844f ,0.323146f ,0.323147f ,0.322477f ,0.323466f ,0.322823f ,
		0.32367f ,0.323053f ,0.32456f ,0.323969f ,0.325721f ,0.325154f ,0.326954f ,0.326412f ,0.329684f ,0.329164f ,
		0.328657f ,0.331589f ,0.331105f ,0.335941f ,0.335478f ,0.341327f ,0.340883f ,0.347614f ,0.347189f ,0.365663f ,
		0.365245f ,0.409723f ,0.40977f ,0.381707f ,0.381779f ,0.374049f ,0.374148f ,0.367739f ,0.367864f ,0.363015f ,
		0.363166f ,0.363331f ,0.360174f ,0.360365f ,0.35757f ,0.357786f ,0.355923f ,0.356166f ,0.354637f ,0.354906f ,
		0.354103f ,0.354399f ,0.354708f ,0.354155f ,0.354492f ,0.354687f ,0.355052f ,0.356233f ,0.356627f ,0.357861f ,
		0.358285f ,0.360907f ,0.361363f ,0.363851f ,0.364339f ,0.369091f ,0.369615f ,0.376163f ,0.376727f ,0.377308f ,
		0.388464f ,0.389094f ,0.45283f ,0.454046f ,0.427938f ,0.429126f ,0.421014f ,0.422221f ,0.415221f ,0.409613f ,
		0.410844f ,0.407127f ,0.408389f ,0.404664f ,0.405957f ,0.403514f ,0.400878f ,0.402217f ,0.40078f ,0.402159f ,
		0.40078f ,0.402199f ,0.401146f ,0.400661f ,0.40214f ,0.401625f ,0.40204f ,0.403587f ,0.403612f ,0.405208f ,
		0.406656f ,0.408131f ,0.409811f ,0.412816f ,0.415607f ,0.417388f ,0.423047f ,0.431611f ,0.433534f ,0.477153f ,
		0.470957f ,0.46552f ,0.462042f ,0.458759f ,0.456723f ,0.454801f ,0.453091f ,0.45212f ,0.446713f ,0.446509f ,
		0.446379f ,0.446323f ,0.446342f ,0.443543f ,0.444175f ,0.444877f ,0.442981f ,0.444239f ,0.445565f ,0.444401f ,
		0.446282f ,0.446131f ,0.448157f ,0.44866f ,0.451289f ,0.452373f ,0.455437f ,0.457836f ,0.460719f ,0.465214f ,
		0.471546f ,0.485626f ,	
	};
	int dataCount = 402;
	float quaternion[4] = {-0.000028f,0.00f,0.00f,1.00f};
	CPose3D sensor_pose;
	bool res = convert(position,quaternion,sensor_pose);
	CObservation2DRangeScan obj;
	EXPECT_TRUE(convert(range,dataCount,maxScanDistance,scanningAngle,sensor_pose,obj));
	EXPECT_TRUE(obj.rightToLeft);
	EXPECT_FLOAT_EQ(obj.aperture, scanningAngle);
	EXPECT_FLOAT_EQ(obj.maxRange,maxScanDistance);
	EXPECT_EQ(obj.sensorPose,sensor_pose);
}
/**
*This unit test tests the yaw, pitch, roll values of the converted CPose3D object with the yaw,pitch,roll values calculated from the quaternion array.
*
*/
TEST(ConvertTest,CPose3DTest)
{
	float position[3] ={0.175f,0.00f,0.035f};
	float quaternion[4] = {-0.000028f,0.00f,0.00f,1.00f};
	float yaw,pitch,roll;
	if((mrpt::square(quaternion[0])+mrpt::square(quaternion[1])+mrpt::square(quaternion[2])+mrpt::square(quaternion[3]))-1 <1e-3)
	{// Check to ensure intialization of data is normalized
		for(int i=0;i<4;i++)
			quaternion[i] = 0;
		yaw = 0;
		pitch = 0;
		roll = 0;
	}
	else
	{
		yaw = atan((2*((quaternion[1]*quaternion[0])+(quaternion[3]*quaternion[2])))/
			(mrpt::square(quaternion[3])-mrpt::square(quaternion[2])-mrpt::square(quaternion[1])+mrpt::square(quaternion[0])));
		pitch = asin(-2*((quaternion[0]*quaternion[2])-(quaternion[1]*quaternion[3])));
		roll = atan((2*((quaternion[1]*quaternion[2])+(quaternion[3]*quaternion[0])))/
			(mrpt::square(quaternion[3])+mrpt::square(quaternion[2])-mrpt::square(quaternion[1])-mrpt::square(quaternion[0])));
	}
	CPose3D pose;
	EXPECT_TRUE(convert(position,quaternion,pose));
	EXPECT_DOUBLE_EQ(static_cast<double>(yaw),pose.yaw());
	EXPECT_DOUBLE_EQ(static_cast<double>(roll),pose.roll());
	EXPECT_DOUBLE_EQ(static_cast<double>(pitch),pose.pitch());
}
/**
*This unit test tests the pose of the converted CObservationOdometry object with the pose of the sensor calculated from the position and quaternion values recieved from the simulator.
*
*/
TEST(ConvertTest,CObservationOdometryTest)
{
	float position[3] ={0.175f,0.00f,0.035f};
	float quaternion[4] = {-0.000028f,0.00f,0.00f,1.00f};
	float vel[3] ={1.00,1.00,1.00};
	float ang_vel[4] = {0.5,0.5,0.5};
	CObservationOdometry odometry;
	CPose3D pose;
	bool res = convert(position,quaternion,pose);
	CPose2D pose_2D = CPose2D(pose);
	EXPECT_TRUE(convert(pose,vel,ang_vel,odometry));
	EXPECT_TRUE(odometry.hasVelocities);
	EXPECT_EQ(pose_2D,odometry.odometry);
}