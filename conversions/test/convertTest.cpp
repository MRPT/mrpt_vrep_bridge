#include <gtest/gtest.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3D.h>
#include "mrpt_bridge/vrep_conversion.h"
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
	float position[3] ={1.00,1.00,1.00} ,scanningAngle = 4.18 ,maxScanDistance = 5;
	float range[] = {2.00,2.00,1.00};
	int dataCount = 3;
	float quaternion[4] = {0.5,0.5,0.5,0.5};
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
	float position[3] ={1.00,1.00,1.00};
	float quaternion[4] = {0.5,0.5,0.5,0.5};
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
	float position[3] ={1.00,1.00,1.00};
	float quaternion[4] = {0.5,0.5,0.5,0.5};
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