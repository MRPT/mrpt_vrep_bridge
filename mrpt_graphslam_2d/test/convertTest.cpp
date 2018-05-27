#include <gtest/gtest.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#define __STDC_FORMAT_MACROS  
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3D.h>
#include "mrpt_bridge/vrep_conversion.h"
extern "C" {
    #include "extApi.h"
}
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace vrep_bridge;


TEST(ConvertTest, CObservation2DRangeScanTest)
{
	
	simxFloat position[3] ={1.00,1.00,1.00} ,scanningAngle = 4.18 ,maxScanDistance = 5;
    simxUChar* dataSignal;
    float range[] = {2.00,2.00,1.00};
    simxFloat quaternion[4] = {0.5,0.5,0.5,0.5};
    CPose3D sensor_pose;
    bool res = convert(position,quaternion,sensor_pose);
    CObservation2DRangeScan obj;
    EXPECT_TRUE(convert(range,maxScanDistance,scanningAngle,sensor_pose,obj));
    EXPECT_TRUE(obj.rightToLeft);
	EXPECT_EQ(obj.aperture, scanningAngle);
	EXPECT_EQ(obj.maxRange,maxScanDistance);
	EXPECT_EQ(obj.sensorPose,sensor_pose);
}

TEST(ConvertTest,CPose3DTest){
	simxFloat position[3] ={1.00,1.00,1.00};
	simxFloat quaternion[4] = {0.5,0.5,0.5,0.5};
	CPose3D pose;
	EXPECT_TRUE(convert(position,quaternion,pose));
	float pitch = asin(-2*((quaternion[0]*quaternion[2])-(quaternion[1]*quaternion[3])));
	EXPECT_EQ((double)pitch,pose.pitch());
}

TEST(ConvertTest,CObservationOdometryTest){
	simxFloat position[3] ={1.00,1.00,1.00};
	simxFloat quaternion[4] = {0.5,0.5,0.5,0.5};
	simxFloat vel[3] ={1.00,1.00,1.00};
	simxFloat ang_vel[4] = {0.5,0.5,0.5};
	CObservationOdometry odometry;
	CPose3D pose;
	bool res = convert(position,quaternion,pose);
	CPose2D pose_2D = CPose2D(pose);
	EXPECT_TRUE(convert(pose,vel,ang_vel,odometry));
	EXPECT_TRUE(odometry.hasVelocities);
	EXPECT_EQ(pose_2D,odometry.odometry);
}