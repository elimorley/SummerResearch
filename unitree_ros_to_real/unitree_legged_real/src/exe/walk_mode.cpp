/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#include <iostream>
#include <cstring>
#include <string>
#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include "std_msgs/Int32.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

int cmd_type = 0;

// class Listen
// {
//     public:
//     string x;

//     void chatterCallback(const std_msgs::Int32::ConstPtr& msg);

// };
    
template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}


//void Listen::chatterCallback(const std_msgs::String::ConstPtr& msg)
void chatterCallback(const std_msgs::Int32::ConstPtr& msg)
{
    cmd_type = msg->data;
}

template<typename TCmd, typename TState, typename TLCM>

int mainHelper(int argc, char *argv[], TLCM &roslcm)
    {
        std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
                << "Make sure the robot is standing on the ground." << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();

        ros::NodeHandle n;
        ros::Rate loop_rate(500);

        // SetLevel(HIGHLEVEL);
        long motiontime = 0;
        TCmd SendHighLCM = {0};
        TState RecvHighLCM = {0};
        unitree_legged_msgs::HighCmd SendHighROS;
        unitree_legged_msgs::HighState RecvHighROS;

        roslcm.SubscribeState();

        pthread_t tid;
        pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

        while (ros::ok()){
            std::cout<<"current cmd = " << cmd_type<<endl;
            motiontime = motiontime+2;
            roslcm.Get(RecvHighLCM);
            RecvHighROS = ToRos(RecvHighLCM);

            SendHighROS.mode = 0;      
            SendHighROS.gaitType = 0;
            SendHighROS.speedLevel = 0;
            SendHighROS.footRaiseHeight = 0;
            SendHighROS.bodyHeight = 0;
            SendHighROS.euler[0]  = 0;
            SendHighROS.euler[1] = 0;
            SendHighROS.euler[2] = 0;
            SendHighROS.velocity[0] = 0.0f;
            SendHighROS.velocity[1] = 0.0f;
            SendHighROS.yawSpeed = 0.0f;
            SendHighROS.reserve = 0;
            
            //cmd_type 0 = stay stationary
            //cmd_type 1 = walking forward
            //cmd_type 2 = walk and turn clockwise
            //cmd_type 3 = walk and turn counter clockwise
            //cmd_type 4 = aruco marker not in view so turn stationary


            if(motiontime > 0 && cmd_type == 0){
                SendHighROS.mode = 1;
            }
            if(motiontime > 0 && cmd_type == 1){
                SendHighROS.mode = 2;
                SendHighROS.gaitType = 1;
                SendHighROS.velocity[0] = 0.2f; // -1  ~ +1
                SendHighROS.bodyHeight = 0.1;
            }
            if(motiontime > 0 && cmd_type == 2){
                SendHighROS.mode = 2;
                SendHighROS.gaitType = 2;
                SendHighROS.velocity[0] = 0.1f; // -1  ~ +1
                SendHighROS.bodyHeight = 0.1;
                SendHighROS.yawSpeed = 0.2;
                SendHighROS.footRaiseHeight = 0.1;
            }
            if(motiontime > 0 && cmd_type == 3){
                SendHighROS.mode = 2;
                SendHighROS.gaitType = 2;
                SendHighROS.velocity[0] = 0.1f; // -1  ~ +1
                SendHighROS.bodyHeight = 0.1;
                SendHighROS.yawSpeed = -0.2;
                SendHighROS.footRaiseHeight = 0.1;
            }
            if(motiontime > 0 && cmd_type == 4){
                SendHighROS.mode = 2;
                SendHighROS.gaitType = 1;
                //SendHighROS.velocity[0] = 0.1f; // -1  ~ +1
                SendHighROS.bodyHeight = 0.1;
                SendHighROS.yawSpeed = 1;
                SendHighROS.footRaiseHeight = 0.1;
            }
            SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
            roslcm.Send(SendHighLCM);
            ros::spinOnce();
            loop_rate.sleep(); 
        }
        return 0;
    }


int main(int argc, char *argv[]){
    

    
    ros::init(argc, argv, "listener_1");
    ros::NodeHandle n;
    //ros::Subscriber sub = n.subscribe<std_msgs::Int32>("chatter", 1000,&Listen::chatterCallback &listen);
    ros::Subscriber sub = n.subscribe<std_msgs::Int32>("chatter", 1000, chatterCallback);
    ros::init(argc, argv, "walk_ros_mode");

    UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    
}