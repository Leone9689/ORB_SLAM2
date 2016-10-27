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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/PangolinViewer.h"
#include"../../../include/Viewer.h"

#include "ros_viewer.hpp"

using namespace std;

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM2::System* pSLAM, bool publish_frame):mpSLAM(pSLAM){
	_publish_frame = publish_frame;
	_publish_pose = false;
  }
  
  void GrabImage(const sensor_msgs::ImageConstPtr& msg);
  void SaveMapCallback(const std_msgs::String::ConstPtr& msg);

  ORB_SLAM2::System* mpSLAM;
  bool _publish_frame, _publish_pose;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    bool use_pango_viewer, publish_frame, publish_pose;
	cerr << "## ORB ROS parameters:" << endl;
    ros::param::param<bool>("orb_use_pango_viewer", use_pango_viewer, true);
    cerr << "# orb use pango viewer " << use_pango_viewer << endl;
	ros::param::param<bool>("orb_publish_frame", publish_frame, true);
    cerr << "# orb publish frame " << publish_frame << endl;
	ros::param::param<bool>("orb_publish_pose", publish_pose, true);
    cerr << "# orb publish pose " << publish_pose << endl;

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::Viewer *viewer;
	if (use_pango_viewer)
	  viewer = new ORB_SLAM2::PangolinViewer(argv[2]);
	else
	  viewer = new ORB_SLAM2::RosViewer();
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, viewer);

    ImageGrabber igb(&SLAM, publish_frame);
    
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub1 = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);
	ros::Subscriber sub2 = nodeHandler.subscribe("/ORB_SLAM2/save_map", 1000, &ImageGrabber::SaveMapCallback, &igb);

	ros::spin();
    
	// Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::SaveMapCallback(const std_msgs::String::ConstPtr& msg) {
  
  cerr << "Saving to " << msg->data.c_str() << endl;
  mpSLAM->SaveMap(msg->data.c_str());

}


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //cv::Mat cam_pose = 
	mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
}


