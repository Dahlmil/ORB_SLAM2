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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "Converter.h"

#include <System.h>
#include <sys/time.h>

using namespace cv;
using namespace std;
using namespace ORB_SLAM2;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

long GetCurrentTime(void);

void QuaternionToeularangle(float &q0, float &q1, float &q2, float &q3);

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl
             << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3]) + "/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    VideoCapture inputVideo(0);
    // Main loop
    cv::Mat im;
    inputVideo >> im;
    while (1)
    {
        // Read image from file
        inputVideo >> im;

        double tframe = GetCurrentTime();

        if (im.empty())
        {
            return 1;
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
            cout << "------------------" << endl;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void QuaternionToeularangle(float &q0, float &q1, float &q2, float &q3)
{
    // cout << "------------------------" << endl;
    // cout << q0 << " " << q1 << " " << q2 << " " << q3 << endl;
    double roll_x, pitch_y, yaw_z;
    // ,yaw_z_angle;
    yaw_z=std::atan2(2*(q3*q2+q0*q1),1-2*(q1*q1+q2*q2)) * 57.29;
    pitch_y=std::asin(2*(q3*q1-q0*q2)) * 57.29;
    roll_x=std::atan2(2*(q3*q0+q1*q2),1-2*(q1*q1+q0*q0)) * 57.29;
    // roll_x = (std::asin(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)*57.29);
    // cout << "[" << roll_x << "]" << endl;
    // cout << "------------------------" << endl;
    cout <<"[" <<roll_x <<"; " << pitch_y <<"; "<<yaw_z << "]" <<endl;

    // cout<<"roll_x="<<roll_x<<endl;
    // cout<<"pitch_y="<<pitch_y<<endl;
    // cout<<"yaw_z="<<yaw_z<<endl;
}

long GetCurrentTime(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f, s0);
    getline(f, s0);
    getline(f, s0);

    while (!f.eof())
    {
        string s;
        getline(f, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
