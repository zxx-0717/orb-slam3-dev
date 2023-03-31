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

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/videoio.hpp>
#include<opencv2/opencv.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);
                
#define Image_Width 1280
#define Image_Height 720

int main(int argc, char **argv)
{  
    if(argc < 4 || argc > 5)
    {
        cerr << endl << "Usage: ./stereo_tx1 path_to_vocabulary path_to_settings video_path (trajectory_file_name)" << endl;

        return 1;
    }
   
    
    bool bFileName= (argc == 5);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    std::string video_path = argv[3];

    // Load all sequences:
    int num_seq = 1;
    int seq;
    vector< vector<string> > vstrImageLeft;
    vector< vector<string> > vstrImageRight;
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;

    vstrImageLeft.resize(num_seq);
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);
    int tot_images = 0;

    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        std::string pathTimeStamps("/home/ubuntu/visual_localization/ORB_SLAM3/Examples/Stereo/EuRoC_TimeStamps/MH01.txt");

        string pathCam0 = "/home/ubuntu/visual_localization/ORB_SLAM3/Examples/euroc_dataset/MH01/mav0/cam0/data";
        string pathCam1 = "/home/ubuntu/visual_localization/ORB_SLAM3/Examples/euroc_dataset/MH01/mav0/cam1/data";

        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);
    std::cout << std::endl << "-----------------" << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO, true);

    // open Stereo video capture
    cv::VideoCapture cap;
    // cap.open(0);
    cap.open(video_path.c_str());
    cap.set(cv::CAP_PROP_FRAME_WIDTH, Image_Width*2);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,Image_Height);
    if (!cap.isOpened())
    {
        printf("Failed to open video capture.\n");
    }
    else
    {
        printf("Open Video capture.\n");
    }

    // process of video frame
    cv::Mat frame, frame_left, frame_right;
    int key = 0;

    double timeStamp = 0;
    std::cout.precision(17);

    while(1)
    {
        cap >> frame;
        // frame = cv::imread("/home/ubuntu/visual_localization/ORB_SLAM3/Examples/Stereo/2.jpg", cv::IMREAD_UNCHANGED);
        if (frame.empty())
        {
            std::cout << "The end of Video" << std::endl;
            break;
        }
        frame_left = frame(cv::Rect(0,0,Image_Width,Image_Height));
        frame_right = frame(cv::Rect(Image_Width,0,Image_Width,Image_Height));

        if (frame_left.empty())
        {
            std::cerr << std::endl << "Failed to load image left" << std::endl;
            return 1;
        }
        if (frame_right.empty())
        {
            std::cerr << std::endl << "Failed to load image right" << std::endl;
            return 1;
        }
        cv::waitKey(100);

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        timeStamp = t1.time_since_epoch().count();
        std::cout << "timeStamp: " << timeStamp << std::endl;
        SLAM.TrackStereo(frame_left,frame_right,timeStamp);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::cout << "Track time: " 
        << (int)std::chrono::duration_cast<std::chrono::duration<double,std::milli>>(t2-t1).count() <<  " ms." << std::endl;
    }
   
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}