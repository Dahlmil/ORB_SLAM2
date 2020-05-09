/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <algorithm>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <ros/ros.h>

// #include "../../../include/System.h"
#include "Converter.h"
#include "System.h"

#include "Path2RVIZ.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

// #define TUM_PATH "/home/dm/CodeBase/ORB_SLAM2_Master/Examples/Monocular/TUM.yaml"

#define TUM_PATH "/home/dm/catkin_ws/SLAM_YAML/TUM.yaml"
#define VOC_PATH "/home/dm/CodeBase/ORB_SLAM2_Master/Vocabulary/ORBvoc.bin"
#define VIDEO_MODE 0

long GetCurrentTime(void);
inline bool FileExists(const std::string &name);

void SLAM2CameraImage(void);
void SLAM2UsbCam(void);
void SLAM2DateSet(void);

/**
 * @description: odom数据回调函数
 * @param {type} 
 * @return: 
 */
void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom);

/**
 * @description: img数据回调函数
 * @param {type} 
 * @return: 
 */
class ImageGrabber
{
public:
    ORB_SLAM2::System *mpSLAM;
    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabImage(const sensor_msgs::ImageConstPtr &msg);

};

RVIZ_POS posChassis;
RVIZ_POS posSLAM;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if (argc != 2)
    {
        cout << "mode?" << endl;
        return 1;
    }

    cout << " argv[1] " << argv[1] << endl;

    string strArgv = argv[1];
    if (!strArgv.compare("0"))
    {
        SLAM2DateSet();
    }
    else if (!strArgv.compare("1"))
    {
        SLAM2UsbCam();
    }
    else if (!strArgv.compare("2"))
    {
        SLAM2CameraImage();
    }

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

    if ((Tcw.cols == 4) && (Tcw.rows == 4))
    {
        Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        Mat Twc = -Rwc * Tcw.rowRange(0, 3).col(3);
        vector<float> q = Converter::toQuaternion(Rwc);

        posSLAM.x = Twc.at<float>(0, 0) * 100;
        posSLAM.y = Twc.at<float>(1, 0) * 100;
        posSLAM.z = Twc.at<float>(2, 0) * 100;

        cout << "X " << posSLAM.x << endl;
        cout << "Y " << posSLAM.y << endl;
        cout << "Z " << posSLAM.z << endl;
        cout << "------------------" << endl;

    }
}

inline bool FileExists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

long GetCurrentTime(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

void SLAM2UsbCam(void)
{
    ORB_SLAM2::System SLAM(VOC_PATH, TUM_PATH, ORB_SLAM2::System::MONOCULAR, true);
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/usb_cam/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void SLAM2CameraImage(void)
{
    memset(&posChassis, 0, sizeof(RVIZ_POS));
    memset(&posSLAM, 0, sizeof(RVIZ_POS));

    ORB_SLAM2::System SLAM(VOC_PATH, TUM_PATH, ORB_SLAM2::System::MONOCULAR, true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle camHandler;
    ros::Subscriber camSub = camHandler.subscribe("camera/image", 1, &ImageGrabber::GrabImage, &igb);

    ros::NodeHandle odomHandler;
    ros::Subscriber odomSub = odomHandler.subscribe<nav_msgs::Odometry>("/odom", 10, OdomCallback);

    ros::NodeHandle n;

    ros::Publisher slamPathPub = n.advertise<nav_msgs::Path>("SLAM_Path", 1, true);
    nav_msgs::Path slamPath;
    slamPath.header.stamp = ros::Time::now();
    slamPath.header.frame_id = "map";

    ros::Publisher chassisPathPub = n.advertise<nav_msgs::Path>("Chassis_Path", 1, true);
    nav_msgs::Path chassisPath;
    chassisPath.header.stamp = ros::Time::now();
    chassisPath.header.frame_id = "map";

    double xSLAM = 0.0;
    double ySLAM = 0.0;
    double xChassis = 0.0;
    double yChassis = 0.0;
    double th = 0.0;

    ros::Publisher slamMarkerPub = n.advertise<visualization_msgs::Marker>("SLAM_Marker", 1);
    visualization_msgs::Marker slamMarker;
    slamMarker.color.r = 1.0f;
    slamMarker.color.g = 0.0f;
    slamMarker.color.b = 1.0f;
    slamMarker.color.a = 1.0;

    ros::Publisher chassisMarkerPub = n.advertise<visualization_msgs::Marker>("Chassis_Marker", 1);
    visualization_msgs::Marker chassisMarker;
    chassisMarker.color.r = 0.0f;
    chassisMarker.color.g = 1.0f;
    chassisMarker.color.b = 1.0f;
    chassisMarker.color.a = 1.0;

    ros::Rate loop_rate(1000);

    unsigned long lConunt = 0;
    while (ros::ok())
    {
        xSLAM = posSLAM.x;
        ySLAM = posSLAM.y;

        xChassis = posChassis.x;
        yChassis = posChassis.y;

        DrawPath(xSLAM, ySLAM, th, slamPathPub, slamPath);
        DrawPath(xChassis, yChassis, th, chassisPathPub, chassisPath);

        DrawMarker(xSLAM, ySLAM, slamMarkerPub, slamMarker);
        DrawMarker(xChassis, yChassis, chassisMarkerPub, chassisMarker);

        ros::spinOnce(); // check for incoming messages

        loop_rate.sleep();

        lConunt++;
    }

    // ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void SLAM2DateSet(void)
{
    ORB_SLAM2::System SLAM(VOC_PATH, TUM_PATH, ORB_SLAM2::System::MONOCULAR, true);
    int iCount = 0;
    Mat im;
    while (1)
    {
        iCount++;
        ostringstream strPath;
        strPath << "/home/dm/CodeBase/SLAM_DATA/img/img" << iCount << ".jpg";
        cout << strPath.str() << endl;
        if (!FileExists(strPath.str()))
        {
            cout << "File NULL" << endl;
            break;
        }
        im = cv::imread(strPath.str());

        double tframe = GetCurrentTime();

        if (im.empty())
        {
            return;
        }

        // Pass the image to the SLAM system

        Mat Tcw = SLAM.TrackMonocular(im, tframe);
        // cout << Tcw.cols << " " << Tcw.rows << endl;
        if ((Tcw.cols == 4) && (Tcw.rows == 4))
        {
            Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            Mat Twc = -Rwc * Tcw.rowRange(0, 3).col(3);
            vector<float> q = Converter::toQuaternion(Rwc);
            cout << "X " << Twc.at<float>(0, 0) * 100 << endl;
            cout << "Y " << Twc.at<float>(1, 0) * 100 << endl;
            cout << "Z " << Twc.at<float>(2, 0) * 100 << endl;
            cout << "-------------------" << endl;
        }

        usleep(10 * 1000);
    }
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    // ROS_INFO("odom %.3lf %.3lf\n", odom->pose.pose.position.x, odom->pose.pose.position.y);
    posChassis.x = odom->pose.pose.position.x;
    posChassis.y = odom->pose.pose.position.y;

}
