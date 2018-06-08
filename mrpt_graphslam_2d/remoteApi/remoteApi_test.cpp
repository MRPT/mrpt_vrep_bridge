#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "mrpt/obs/CObservation2DRangeScan.h"
#include "mrpt/obs/CObservationOdometry.h"
#include "mrpt/poses/CPose3D.h"
#include "../../conversions/include/vrep_conversion.h"
extern "C" 
{
    #include "extApi.h"
}
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace vrep_bridge;

int main()
{   /**
    *This is a sample VREP remote api which fetches the following data from the simulator
    *- <b>object handle[]</b> -> Using simxGetObjectHandle.
    *- <b>object position</b> -> Using simxGetObjectPosition
    *- <b>object velocity</b> -> Using simxGetObjectVelocity
    *- <b>object quaternion</b> -> Using simxGetObjectQuaternion
    *
    * <i> This api also makes use of the conversion methods defined in ../../conversions/vrep_conversion.cpp<i> 
    */
    simxFinish(-1);//close all existing connections.
    int clientID=simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
    simxInt handle;
    simxFloat position[3],scanningAngle,maxScanDistance;
    simxUChar* dataSignal;
    int dataSignalSize,dataCount;
    simxFloat quaternion[4],vel[3],angularvel[3];
    if (clientID!=-1)
    {
        printf("Simulation started \n");
        while (simxGetConnectionId(clientID)!=-1)
        {
            simxGetObjectHandle(clientID,"fastHokuyo",&handle,simx_opmode_streaming);//get object handlesimxGetObjectPosition(clientID,handle,-1,position,simx_opmode_streaming); // get object position
            simxGetObjectPosition(clientID,handle,-1,position,simx_opmode_streaming); // get object orientation(returns euler angles)
            simxGetObjectQuaternion(clientID,handle,-1,quaternion,simx_opmode_streaming);
            simxGetStringSignal(clientID,"measuredDataAtThisTime",&dataSignal,&dataSignalSize,simx_opmode_streaming);
            simxGetFloatSignal(clientID,"scanningAngle",&scanningAngle,simx_opmode_streaming);
            simxGetFloatSignal(clientID,"maxScanDistance",&maxScanDistance,simx_opmode_streaming);
            simxGetObjectVelocity(clientID,handle,vel,angularvel,simx_opmode_streaming);
            CPose3D sensor_pose;
            bool pose_convert = convert(position,quaternion,sensor_pose);
            if (simxGetStringSignal(clientID,"measuredDataAtThisTime",&dataSignal,&dataSignalSize,simx_opmode_buffer)==simx_error_noerror)
            {
                dataCount=dataSignalSize/12;
                float range[dataCount];
                for (int i=0;i<dataCount;i++)
                {
                    float x =((float*)dataSignal)[3*i];
                    float y =((float*)dataSignal)[3*i+1];
                    range[i] = sqrt(x*x + y*y);
                }
                CObservation2DRangeScan obj;
                bool rangescan_convert = convert(range,dataCount,maxScanDistance,scanningAngle,sensor_pose,obj);
                printf("Aperture : %f\n",obj.aperture );
            }
            CObservationOdometry odometry;
            bool odometry_convert = convert(sensor_pose, vel, angularvel,odometry);
            printf("Handle : %u\n",handle);
            printf("Laser Position : (%f,%f,%f) \n",position[0],position[1],position[2]);
            extApi_sleepMs(100);
        }
        simxFinish(clientID);
    }
    return(0);
}

