#include <stdio.h>
#include <stdlib.h>
#include <math.h>

extern "C" {
    #include "extApi.h"
}

int main()
{
    simxFinish(-1); //close all existing connections.

    int clientID=simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
    simxInt handle;
    simxFloat position[3];
    simxFloat orient[3];
    float x[2] ={0,0.1};
    float y[2] ={0,0.1};

    if (clientID!=-1)
    {
        printf("Simulation started \n");

        while (simxGetConnectionId(clientID)!=-1) //checl for successful connection
        {   
            for(int i=0;i<2;i++){
                for(int j=0;j<2;j++){
                    for(int k=0;k<2;k++){
                        simxGetObjectHandle(clientID,"bubbleRob",&handle,simx_opmode_streaming); //get object handle
                        simxGetObjectPosition(clientID,handle,-1,position,simx_opmode_streaming); // get object position
                        simxGetObjectOrientation(clientID,handle,-1,orient,simx_opmode_streaming); // get object orientation(returns euler angles)

                        printf("bubbleRob Handle : %u\n",handle);   
                        for(int u =0 ;u<3;u++){
                            printf("bubbleRob Position %i : %f\n",u,position[u]);
                            printf("bubbleRob Orientation %i : %f\n",u,orient[u]);    
                            printf("\n");
                        }

                        position[0] = x[j];
                        position[1] = y[k];
                        position[2] = 0;
                    
                        simxSetObjectPosition(clientID,handle,-1,position,simx_opmode_streaming);
                        simxSetObjectOrientation(clientID,handle,-1,orient,simx_opmode_streaming); 
                        printf("New position and orientation set. \n");                       
                        extApi_sleepMs(1000);            
                    }
                }
            }
            
        }
        simxFinish(clientID);
    }
    return(0);
}

