#include <gtest/gtest.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#define __STDC_FORMAT_MACROS  
#include <mrpt/obs/CObservation2DRangeScan.h>
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
    
    CPose3D sensor_pose(position[0],position[1],position[2]);
    CObservation2DRangeScan obj;
    EXPECT_TRUE(convert(range,maxScanDistance,scanningAngle,sensor_pose,obj));
    EXPECT_TRUE(obj.rightToLeft);
	EXPECT_EQ(obj.aperture, scanningAngle);
	EXPECT_EQ(obj.maxRange,maxScanDistance);
	EXPECT_EQ(obj.sensorPose,sensor_pose);
}

