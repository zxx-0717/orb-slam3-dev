/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "/workspaces/capella_orb_slam3/src/Orbbec_SDK/Example/cpp/window.hpp"
#include "/workspaces/capella_orb_slam3/src/Orbbec_SDK/Example/cpp/conio.h"


#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <System.h>
#include "libobsensor/ObSensor.hpp"

extern "C" {
#include <stdlib.h>
#include <libobsensor/h/Error.h>
#include <libobsensor/h/Frame.h>
#include <libobsensor/h/ObTypes.h>
#include <libobsensor/h/Pipeline.h>
#include <libobsensor/h/StreamProfile.h>
#include <libobsensor/h/Device.h>
#include <libobsensor/h/Sensor.h>
}

using namespace std;

#define frame_width 640
#define frame_height 360
#define frame_fps 30
#define ESC 27

ob_error *   error  = NULL;     // 用于SDK 接口错误信息返回

std::vector<cv::Mat> processFrames(std::vector<std::shared_ptr<ob::Frame>> frames);


int main(int argc, char **argv) {

    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./mono_inertial_realsense_D435i path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;
    bool bFileName = false;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
        bFileName = true;
    }

    ob::Pipeline pipe;
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
    auto colorProfiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
    if(colorProfiles) {
        colorProfile = colorProfiles->getVideoStreamProfile(frame_width, frame_height, OB_FORMAT_RGB, frame_fps);
    }
    // std::cout << "color format: " << colorProfile->format() 
    // << ", width: " << colorProfile->width() << ", height: " << colorProfile->height() 
    // << ", fps: " << colorProfile->fps() << std::endl;
    

    std::shared_ptr<ob::VideoStreamProfile> depthProfile  = nullptr;
    auto  depthProfiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
    if(depthProfiles) {
        depthProfile = depthProfiles->getVideoStreamProfile(frame_width, frame_height, OB_FORMAT_Y11, frame_fps);
    }
    // std::cout << "depth format: " << depthProfile->format() 
    // << ", width: " << depthProfile->width() << ", height: " << depthProfile->height() 
    // << ", fps: " << depthProfile->fps() << std::endl;

    config->enableStream(colorProfile);
    config->enableStream(depthProfile);
    config->setAlignMode(ALIGN_D2C_HW_MODE);

    try {
        pipe.start(config);
    }
    catch(ob::Error &e) {
        std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    }

    if (pipe.getDevice()->isPropertySupported(OB_PROP_COLOR_MIRROR_BOOL, OB_PERMISSION_WRITE))
    {
        pipe.getDevice()->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, true);
    }

    if (pipe.getDevice()->isPropertySupported(OB_PROP_DEPTH_MIRROR_BOOL, OB_PERMISSION_WRITE))
    {
        pipe.getDevice()->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, true);
    }

    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();
    cout << "slam imageScale : " << imageScale << endl;

    Window app("SyncAlignViewer", colorProfile->width(), colorProfile->height());
    int key = -1;
    uint64_t timeStamp;
    while(!SLAM.isShutDown())
    {
        if(kbhit()) 
        {
            int key = getch();
            // 按ESC键退出
            if(key == ESC) 
            {
                break;
            }
        }
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr)
        {
            continue;
        }

        auto frames_color = frameSet->colorFrame();
        auto frames_depth = frameSet->depthFrame();
        // timeStamp = frames_depth->systemTimeStamp();
        // cout << "scale value: --------" << frames_depth->getValueScale() <<endl;
        vector<cv::Mat> mats = processFrames({frames_color, frames_depth});        

        int width = frame_width * imageScale;
        int height = frame_height * imageScale;
        if (mats.size() == 2)
        {
            // cv::imshow("mat[0]", mats[0]);
            // cv::imshow("mat[1]", mats[1]);
            // cv::waitKey(30);
            cout << fixed;
            // cv::resize(mats[0], mats[0], cv::Size(width, height));
            // cv::resize(mats[1], mats[1], cv::Size(width, height));
            chrono::steady_clock::time_point time_start = chrono::steady_clock::now();
            timeStamp = chrono::duration_cast<chrono::microseconds>(time_start.time_since_epoch()).count();
            Sophus::SE3f Tcw_SE3f = SLAM.TrackRGBD(mats[0], mats[1], timeStamp); // Track
            cv::Mat Tcw;
            Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
            cv::eigen2cv(Tcw_Matrix, Tcw);
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            vector<float> euler = ORB_SLAM3::Converter::toEuler(Rwc);
            float position_x, position_y, position_z;
            float orientation_roll,orientation_pitch, orientation_yaw;
            position_x = twc.at<float>(0);
            position_y = twc.at<float>(1);
            position_z = twc.at<float>(2);
            orientation_roll = euler[0];
            orientation_pitch = euler[1];
            orientation_yaw = euler[2];
            cout << "----------------------\n";
            // cout << "position: " << position_x  << ", " << position_y << ", " << position_z << endl; 
            cout << "position: " << Tcw_SE3f.inverse().translation()[0] * 16 << ", " << Tcw_SE3f.inverse().translation()[1] * 16 << ", " << Tcw_SE3f.inverse().translation()[2] * 16 << endl; 
            // cout << "orientation: " << orientation_roll << ", " << orientation_pitch << ", " << orientation_yaw << endl;
            cout << "orientation: " << Tcw_SE3f.inverse().angleX() << ", " << Tcw_SE3f.inverse().angleY() << ", " << Tcw_SE3f.inverse().angleZ() << endl;; 
            chrono::steady_clock::time_point time_end = chrono::steady_clock::now();
            int cost_time = chrono::duration_cast<chrono::milliseconds>(time_end- time_start).count();
            cout << "TrackRGBD costs time: " << cost_time << " ms." << endl;
        }
    }

    std::cout << "*********************slam shutdown.**************************" << std::endl;
    // 停止pipeline
    pipe.stop();
    SLAM.Shutdown();
    if (bFileName)
    {
        SLAM.SaveTrajectoryEuRoC("rgbd_dabaiDCW.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("rgbd_dabaiDCW_key.txt");
    }
}


std::vector<cv::Mat> processFrames(std::vector<std::shared_ptr<ob::Frame>> frames) {
        std::vector<cv::Mat> mats;
        for(auto frame: frames) {
            if(frame == nullptr || frame->dataSize() < 1024) {
                continue;
                // return mats;
            }
            auto videoFrame = frame->as<ob::VideoFrame>();
            // std::cout << "videoFrame->type(): " << videoFrame->type() << ", videoFrame->format(): " << videoFrame->format() << std::endl;

            cv::Mat rstMat;

            if(videoFrame->type() == OB_FRAME_COLOR && videoFrame->format() == OB_FORMAT_MJPG) {
                cv::Mat rawMat(1, videoFrame->dataSize(), CV_8UC1, videoFrame->data());
                rstMat = cv::imdecode(rawMat, 1);
            }
            else if(videoFrame->type() == OB_FRAME_COLOR && videoFrame->format() == OB_FORMAT_NV21) {
                cv::Mat rawMat(videoFrame->height() * 3 / 2, videoFrame->width(), CV_8UC1, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_NV21);
            }
            else if(videoFrame->type() == OB_FRAME_COLOR && (videoFrame->format() == OB_FORMAT_YUYV || videoFrame->format() == OB_FORMAT_YUY2)) {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_YUY2);
            }
            else if(videoFrame->type() == OB_FRAME_COLOR && videoFrame->format() == OB_FORMAT_RGB) {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC3, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_RGB2BGR);
            }
            else if(videoFrame->type() == OB_FRAME_COLOR && videoFrame->format() == OB_FORMAT_UYVY) {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_UYVY);
            }
            else if(videoFrame->format() == OB_FORMAT_Y16 || videoFrame->format() == OB_FORMAT_YUYV || videoFrame->format() == OB_FORMAT_YUY2) {
                if(videoFrame->type() == OB_FRAME_DEPTH) {
                }
                else if(videoFrame->type() == OB_FRAME_IR) {
                }
                // IR or Depth Frame
                cv::Mat cvtMat;
                cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_16UC1, videoFrame->data());
                cout << endl << endl;
                cout.fill(0);
                cout.width(16);
                cout << "------------------------------------------------------------------------------------------------" << endl;
                float   scale;
                if(videoFrame->type() == OB_FRAME_DEPTH) {
                    scale = 1.0f / pow(2, videoFrame->pixelAvailableBitSize() - 10);
                    // cout << "format: " << videoFrame->format() << endl;
                    // cout << "Available bit size: " << (int)videoFrame->pixelAvailableBitSize()<< endl;
                    // std::cout << "scale: " << scale << std::endl;
                }
                else {
                    scale = 1.0f / pow(2, videoFrame->pixelAvailableBitSize() - 8);
                }
                // cv::convertScaleAbs(rawMat, cvtMat, scale); // scale for show depth
                cvtMat = rawMat; // without scale
                cvtMat.convertTo(rstMat, CV_32F);
                cv::cvtColor(cvtMat, rstMat, cv::COLOR_GRAY2RGB);
            }
            else if(videoFrame->type() == OB_FRAME_COLOR && videoFrame->format() == OB_FORMAT_UYVY) {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_UYVY);
            }

            if(videoFrame->type() == OB_FRAME_DEPTH) {
            }
            else if(is_ir_frame(videoFrame->type()) && videoFrame->format() == OB_FORMAT_Y8) {
                cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_8UC1, videoFrame->data());

                cv::cvtColor(rawMat, rstMat, cv::COLOR_GRAY2RGB);
            }
            else if(is_ir_frame(videoFrame->type()) && videoFrame->format() == OB_FORMAT_MJPG) {
                cv::Mat rawMat(1, videoFrame->dataSize(), CV_8UC1, videoFrame->data());
                rstMat = cv::imdecode(rawMat, 1);
            }
            else if(is_ir_frame(videoFrame->type())) {
            }
            mats.push_back(rstMat);
        }
        return mats;
    }