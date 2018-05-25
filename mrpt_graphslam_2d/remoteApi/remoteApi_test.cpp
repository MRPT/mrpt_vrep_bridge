#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>  
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/poses/CPose3D.h>
#include "mrpt_bridge/vrep_conversion.h"
extern "C" {
    #include "extApi.h"
}
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace vrep_bridge;

int main()
{
    simxFinish(-1); //close all existing connections.

    int clientID=simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
    simxInt handle;
    simxFloat position[3],scanningAngle,maxScanDistance;
    simxUChar* dataSignal;
    int dataSignalSize,dataCount;
    

    if (clientID!=-1)
    {
        printf("Simulation started \n");

        while (simxGetConnectionId(clientID)!=-1){
            simxGetObjectHandle(clientID,"fastHokuyo",&handle,simx_opmode_streaming); //get object handlesimxGetObjectPosition(clientID,handle,-1,position,simx_opmode_streaming); // get object position
            simxGetObjectPosition(clientID,handle,-1,position,simx_opmode_streaming); // get object orientation(returns euler angles)
            simxGetStringSignal(clientID,"measuredDataAtThisTime",&dataSignal,&dataSignalSize,simx_opmode_streaming);
            simxGetFloatSignal(clientID,"scanningAngle",&scanningAngle,simx_opmode_streaming);
            simxGetFloatSignal(clientID,"maxScanDistance",&maxScanDistance,simx_opmode_streaming);
            if (simxGetStringSignal(clientID,"measuredDataAtThisTime",&dataSignal,&dataSignalSize,simx_opmode_buffer)==simx_error_noerror)
            {
                dataCount=dataSignalSize/12;
                CPose3D sensor_pose(position[0],position[1],position[2]);
                CObservation2DRangeScan obj;
                bool res = convert(dataSignal,dataCount,maxScanDistance,scanningAngle,sensor_pose,obj);
                printf("%d\n",res);
                
            }
            printf("Scanning angle : %f\n",scanningAngle ); 
            printf("Handle : %u\n",handle); 
            printf("Laser Position : (%f,%f,%f) \n",position[0],position[1],position[2]);    
            extApi_sleepMs(100);             
        }
        simxFinish(clientID);
    }
    return(0);
}

