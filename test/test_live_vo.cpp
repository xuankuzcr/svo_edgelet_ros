// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <svo/math_lib.h>
#include <svo/camera_model.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>

#include <svo/slamviewer.h>
#include<thread>

class BenchmarkNode
{
    svo::AbstractCamera* cam_;
    svo::PinholeCamera* cam_pinhole_;
    svo::FrameHandlerMono* vo_;

    SLAM_VIEWER::Viewer* viewer_;
    std::thread * viewer_thread_;

public:
    BenchmarkNode();
    ~BenchmarkNode();
    void runFromFolder();
};

BenchmarkNode::BenchmarkNode()
{
    /*
    cam_pinhole_ = new svo::PinholeCamera(640,480,502.961104,503.651566, 284.978460, 247.527333,
                                          -0.378740,0.133422, -0.001505, -0.001445);
    cam_ = new svo::PinholeCamera(640,480,407.763641, 453.693298, 267.111836,247.958895);
     */
   // cam_ = new svo::PinholeCamera(640,512,8.6204125823337927e+02/2,8.6345560511126575e+02/2, 6.2633572097598585e+02/2, 5.3239736680174201e+02/2, -0.031563,0.165711,0.001507,-0.00083,-0.18942);
   cam_ = new svo::PinholeCamera(640,512,433.9275,434.0748, 320.0464, 260.4712, -0.031563,0.165711,0.001507,-0.00083,-0.18942);


    vo_ = new svo::FrameHandlerMono(cam_);
    vo_->start();

    viewer_ = new SLAM_VIEWER::Viewer(vo_);
    viewer_thread_ = new std::thread(&SLAM_VIEWER::Viewer::run,viewer_);
    viewer_thread_->detach();

}

BenchmarkNode::~BenchmarkNode()
{
    delete vo_;
    delete cam_;
    delete cam_pinhole_;

    delete viewer_;
    delete viewer_thread_;
}

//#define TXTREAD
void BenchmarkNode::runFromFolder()
{

    cv::VideoCapture cap;  // open the default camera
    cap.open("/home/chunran/rosbag2video/test_mono_factory.mp4");
    std::cout<<"111111111111"<<std::endl;

    if (!cap.isOpened())  // check if we succeeded
        return ;

    int img_id = 0;
    for (;;) {

        cv::Mat image;
        cap.read(image);  // get a new frame from camera
    // if(!image.empty())
    // {
        //     std::cout<<"222222222222"<<std::endl;
        assert(!image.empty());
        //cv::resize(image,image,cv::Size(image.cols*0.5,image.rows*0.5),0,0,cv::INTER_LINEAR);
        
        img_id++;

        cv::imshow("origin_image", image);
        if (cv::waitKey(1) >= 0) break;
        if(img_id < 100) continue;

        cv::cvtColor(image,image,CV_BGR2GRAY);
        /*
        cv::Mat unimg;
        cam_pinhole_->undistortImage(image,unimg);
        vo_->addImage(unimg, 0.01*img_id);
         */
        vo_->addImage(image, 0.01*img_id);

        // display tracking quality
        if(vo_->lastFrame() != NULL)
        {
            std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                      << "#Features: " << vo_->lastNumObservations() << " \n";
            //<< "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";
            std::cout<<"Frame pose: "<< vo_->lastFrame()->T_f_w_ <<std::endl;

        }
    //}

    }

    cap.release();
    return;

}


int main(int argc, char** argv)
{


    BenchmarkNode benchmark;
    benchmark.runFromFolder();

    printf("BenchmarkNode finished.\n");
    return 0;
}

