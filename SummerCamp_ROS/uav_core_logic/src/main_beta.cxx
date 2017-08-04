/** @file UAV core logic node
  * @author DreamTale
  * @date Jul 30, 2016
  */
#include "generalHeader.h"
#include "recvInfo.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <uav_vision/DetectInfo.h>
#include <lonely_info/StatCtrl.h>

ros::Publisher pubServo;
ros::Publisher pubOnboard;

using namespace cv;
using namespace std;

// The control status we'll used
// The sort of uav control signals
enum uavCtrlSignal
{
    uavCsMissionStart = 0,
    uavCsGrabOctopus,
    uavCsGrabHippo,
    uavCsTakeOff,
    uavCsMission1,      // Fly to the led area 1 and throw octopus,
                        //  then fly to the landing space
    uavCsMission2,      // Fly to the led area 1 and throw hippo,
                        //  then fly to the landing space
    uavCsMission3,      // Fly to the led area 2 and throw hippo,
                        //  then fly to the landing space
    uavCsMission4,      // Fly to the suduku and throw hippo,
                        //  then fly to the landing space
    uavCsLanding,

    uavCsRequestLanding,
    uavCsRequestCarRelease,
    uavCsFlyReady
};

// The sort of uav mission status
enum uavStatusInfo
{
    uavStReady       = 0x200000, //Changed by zhong
    uavStRunning     = 0x400000,
    uavStError       = 0x800000
};
// The sort of mission
enum uavMissionIndex
{
    uavMiStart = 0,
    uavMiLedThrow,
    uavMiFlyToCar,
    uavMiLandCar
};
// car2uav
enum carCallBackSignal
{
    uavCanLanding = 20,      
    uavCannotLanding,        
    uavCanFly,
    uavCannotFly
};

volatile double droneVelPosInfo[7] = {0.0};
volatile double droneMarkerInfo[4] = {0.0};
volatile int    droneLedStatus = 0;    // 0 ==> null suit pattern
volatile int    globalCarStatus = uavCannotLanding;

int64_t globalCtrlSignal;
int     globalStatusInfo = uavCsMissionStart;

// Landmark:
// Detect the led screen 1
Point3f ledScreen1Detect;
// Mission to throw doll to the screen 1
Point3f ledScreen1Throw;

// Detect the led screen 2
Point3f ledScreen2Detect;
// Mission to throw doll to the screen 2
Point3f ledScreen2Throw;

Point3f carDetect;

Point3f sudukuDetect[9], sudukuThrowDetect[9];

void readParam()
{
    //    memset(sendBuf, 0, BUF_SIZE);
    //    sendBuf[0] = '!';
    //    sendBuf[BUF_SIZE - 1] = '#';

    cv::FileStorage fs;
    fs.open("/home/qy/DronePosParam.yml", cv::FileStorage::READ);

    ledScreen1Detect.x = fs["ledDetectX1"];
    ledScreen1Detect.y = fs["ledDetectY1"];
    ledScreen1Detect.z = fs["ledDetectZ1"];

    ledScreen2Detect.x = fs["ledThrowX1"];
    ledScreen2Detect.y = fs["ledThrowY1"];
    ledScreen2Detect.z = fs["ledThrowZ1"];

    carDetect.x = fs["carDetectX"];
    carDetect.y = fs["carDetectY"];
    carDetect.z = fs["carDetectZ"];
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uavCoreLogicPro");
    ros::NodeHandle nh;
    ros::Rate loopRate(50);

    ros::ServiceClient clientOnboard = nh.serviceClient<lonely_info::StatCtrl>("/uav_ctrl/onboard");
    ros::ServiceClient clientBuffCar = nh.serviceClient<lonely_info::StatCtrl>("/uav_ctrl/buff_car");

    lonely_info::StatCtrl srvOnboard, srvBuffCar, srvHand;

    // Main Mission running ...
    int missionId = 1;
    char command;

    //globalStatusInfo = uavCsGrabHippo;

    while (nh.ok())
    {
        ros::spinOnce();
        loopRate.sleep();
        readParam();
        //globalStatusInfo = uavCsGrabHippo;
        // for test
        // cin >> command;
        // if (command == 'q')
        //     break;
         
        // Start mission rocking...
        if(globalStatusInfo == (uavCsMissionStart))
        {
            srvOnboard.request.ctrlSignal = globalCtrlSignal = uavCsGrabOctopus;
            cout << "## I'll grab octopus. ##" << endl;
            clientOnboard.call(srvOnboard);
            globalStatusInfo = srvOnboard.response.statusInfo;
            if(globalStatusInfo & uavStError)
                cout << "Ops, the drone cannot grab octopus." << endl;
            else
            	cout << "** I've grabed octopus~ **" << endl;
        }
        else if (globalStatusInfo == (uavCsGrabOctopus))
        {
            srvOnboard.request.ctrlSignal = uavCsTakeOff;
            globalStatusInfo = uavStRunning | uavCsTakeOff;
            cout << "## I'll take off ##" << endl;
            clientOnboard.call(srvOnboard);
            globalStatusInfo = srvOnboard.response.statusInfo;
            if(globalStatusInfo & uavStError)
                cout << "Ops, the drone cannot take off " << endl;
            else
            	cout << "** I've taken off~ **" << endl;
        }
        else if(globalStatusInfo == uavCsTakeOff)
        {
            sleep(2);
            srvBuffCar.request.ctrlSignal = uavCsFlyReady;
            clientBuffCar.call(srvBuffCar);

            if(missionId == 1)
            {
                missionId ++;
                srvOnboard.request.ctrlSignal = uavCsMission1;
                globalStatusInfo = uavStRunning | uavCsMission1;
                cout << "## I'll go mission 1 ##" << endl;
                clientOnboard.call(srvOnboard);
                globalStatusInfo = srvOnboard.response.statusInfo;
                if(globalStatusInfo & uavStError)
                    cout << "Ops, the drone cannot go mission 1 " << endl;
                else
            		cout << "** I've completed the mission 1~ **" << endl;
            }
            else if(missionId == 2)
            {
                missionId ++;
                srvOnboard.request.ctrlSignal = uavCsMission2;
                globalStatusInfo = uavStRunning | uavCsMission2;
                cout << "## I'll go mission 2 ##" << endl;
                clientOnboard.call(srvOnboard);
                globalStatusInfo = srvOnboard.response.statusInfo;
                if(globalStatusInfo & uavStError)
                    cout << "Ops, the drone cannot go mission 2 " << endl;
                else
            		cout << "** I've completed the mission 2~ **" << endl;
            }
            else if(missionId <= 5)
            {
            	missionId ++;
                srvOnboard.request.ctrlSignal = uavCsMission3;
                globalStatusInfo = uavStRunning | uavCsMission3;
                cout << "## I'll go mission 3 ##" << endl;
                clientOnboard.call(srvOnboard);
                globalStatusInfo = srvOnboard.response.statusInfo;
                if(globalStatusInfo & uavStError)
                    cout << "Ops, the drone cannot go mission 3 " << endl;
                else
            		cout << "** I've completed the mission 3~ **" << endl;
            }
        }
        else if(globalStatusInfo == uavCsMission1 ||
                globalStatusInfo == uavCsMission2 ||
                globalStatusInfo == uavCsMission3)
        {
            srvBuffCar.request.ctrlSignal = uavCsRequestLanding;
            globalStatusInfo = uavStRunning | uavCsRequestLanding;
            cout << "## I'll request to land ##" << endl;
            clientBuffCar.call(srvBuffCar);
            globalStatusInfo = srvBuffCar.response.statusInfo;
            if(globalStatusInfo != uavCanLanding && globalStatusInfo != uavCannotLanding)
            	globalStatusInfo = uavCannotLanding;
            if(globalStatusInfo & uavStError)
                cout << "Ops, the drone cannot land " << endl;
            else if(globalStatusInfo == uavCanLanding)
           	    cout << "** [1]I will land on the car~ **" << endl;
           	else
           		cout << "** [1]I'll try again~" << endl;

        }
        else if(globalStatusInfo == uavCannotLanding)
        {
            sleep(1);
            srvBuffCar.request.ctrlSignal = uavCsRequestLanding;
            globalStatusInfo = uavStRunning | uavCsRequestLanding;
            cout << "## I request to land again ##" << endl;
            clientBuffCar.call(srvBuffCar);
            globalStatusInfo = srvBuffCar.response.statusInfo;
            if(globalStatusInfo != uavCanLanding && globalStatusInfo != uavCannotLanding)
            	globalStatusInfo = uavCannotLanding;
            if(globalStatusInfo & uavStError)
                cout << "Ops, the drone cannot land " << endl;
            else if(globalStatusInfo == uavCanLanding)
           	    cout << "** [1]I will land on the car~ **" << endl;
           	else
           		cout << "** [1]I'll try again~" << endl;
        }
        else if(globalStatusInfo == uavCanLanding)
        {
            srvOnboard.request.ctrlSignal = uavCsLanding;
            globalStatusInfo = uavStRunning | uavCsLanding;
            cout << "## I'll land  ##" << endl;
            clientOnboard.call(srvOnboard);
            globalStatusInfo = srvOnboard.response.statusInfo;
            if(globalStatusInfo & uavStError)
                cout << "Ops, the drone cannot land " << endl;
            else if(globalStatusInfo == uavCanLanding)
           	    cout << "** I have landed on the car~ **" << endl;
           	else
           		cout << "** I cannot land on the car~" << endl;
        }
        else if(globalStatusInfo == uavCsLanding)
        {
            srvOnboard.request.ctrlSignal = uavCsGrabHippo;
            globalStatusInfo = uavStRunning | uavCsGrabHippo;
            cout << "## I'll grab hippo ##" << endl;
            clientOnboard.call(srvOnboard);
            globalStatusInfo = srvOnboard.response.statusInfo;
            if(globalStatusInfo & uavStError)
                cout << "Ops, the drone cannot land " << endl;
            else 
            	cout << "** I have grabed hippo~" << endl;
        }
        else if (globalStatusInfo == (uavCsGrabHippo))
        {
            srvBuffCar.request.ctrlSignal = uavCsRequestCarRelease;
            globalStatusInfo = uavStRunning | uavCsRequestCarRelease;
            cout << "## I'll request the car to release the doll ##" << endl;
            clientBuffCar.call(srvBuffCar);
            globalStatusInfo = srvBuffCar.response.statusInfo;
            if(globalStatusInfo != uavCanFly && globalStatusInfo != uavCannotFly)
            	globalStatusInfo = uavCsGrabHippo;
            cout << "******##########" <<globalStatusInfo <<endl;
            if(globalStatusInfo & uavStError)
                cout << "Ops, the car cannot release the doll " << endl;
            else
            	cout << "** The car released the hippo" << endl;
        }
        else if(globalStatusInfo == uavCannotFly)
        {
            sleep(1);
            srvBuffCar.request.ctrlSignal = uavCsRequestCarRelease;
            globalStatusInfo = uavStRunning | uavCsRequestCarRelease;
            cout << "## I request the car to release the doll again ##" << endl;
            clientBuffCar.call(srvBuffCar);
            globalStatusInfo = srvBuffCar.response.statusInfo;
            if(globalStatusInfo != uavCanFly && globalStatusInfo != uavCannotFly)
            	globalStatusInfo = uavCannotFly;
            if(globalStatusInfo & uavStError)
                cout << "Ops, the drone cannot take off " << endl;
        }
        else if(globalStatusInfo == uavCanFly)
        {
            srvOnboard.request.ctrlSignal = uavCsTakeOff;
            globalStatusInfo = uavStRunning | uavCsTakeOff;
            cout << "## I'll take off ##" << endl;
            clientOnboard.call(srvOnboard);
            globalStatusInfo = srvOnboard.response.statusInfo;
            if(globalStatusInfo & uavStError)
                cout << "Ops, the drone cannot take off " << endl;
            else
            	cout << "** I will take off now **" << endl;
        }
    }

    return 0;
}
