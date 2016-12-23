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
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include"../../../include/System.h"
#include"../../../include/PangolinViewer.h"
#include"../../../include/Viewer.h"
#include"../../../include/KeyFrame.h"
#include "MapPublisher.hpp"
#include "OdomPublisher.hpp"
#include "ORB_SLAM2/SaveMap.h"
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):subCnt(0),mpSLAM(pSLAM)
    {
    }
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    bool status(ORB_SLAM2::SaveMap::Request &req,ORB_SLAM2::SaveMap::Response &res); 
    int subCnt;                              
    std::map<int,int> cloudSize;             
    ros::NodeHandle n;                       
    
    cv::Mat cloud;    
    ORB_SLAM2::System* mpSLAM;               
    ORB_SLAM2::Frame* currFrame;             
    
    //ROS_ORB_SLAM::CloudPublisher _cloud_pub; 
    ORB_SLAM2::MapPublisher _map_pub;
    ORB_SLAM2::OdomPublisher _odom_pub;
};
bool ImageGrabber::status(ORB_SLAM2::SaveMap::Request  &req, 
                          ORB_SLAM2::SaveMap::Response &res) 
{                                                            
  res.sum = req.a;                                           
  ROS_INFO("request: bool=%d", (bool)req.a);                 
  ROS_INFO("sending back response: [%d]", (bool)res.sum);    
  if(res.sum)                                                
    mpSLAM->SaveMap("Map.map");                                

  return true;                                               
}                                                            


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    bool use_pango_viewer = false;                                 
                                                                
    ros::param::get("~use_pango_viewer",use_pango_viewer);         
    cerr << "## ORB ROS parameters:" << endl;                      
    cerr << "# orb use pango viewer " << use_pango_viewer << endl; 
	
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::Viewer *viewer=NULL;
	if (use_pango_viewer)
	  viewer = new ORB_SLAM2::PangolinViewer(argv[2]);
	else
	  viewer = new ORB_SLAM2::Viewer();
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD, viewer);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
   
    ros::ServiceServer service = nh.advertiseService("save_map", &ImageGrabber::status,&igb); 
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    if (use_pango_viewer) {
      ros::spin();
    }
    else {
      int fps = 10;
      ros::Rate r(fps);
      while (ros::ok()) {
	//igb._frame_pub.refresh();
	ros::spinOnce();
	r.sleep();
      }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //SLAM.SaveMap("Map.map");
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    currFrame = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    
    cloud = cv::Mat::zeros(mpSLAM->GetMap()->GetAllMapPoints().size(), 1, CV_32FC3 ); 
    
    int temp=0;
    for(auto mp: mpSLAM->GetMap()->GetAllMapPoints())    
    {    
      cv::Mat wp = mp->GetWorldPos();  
      cloud.at<cv::Vec3f>(temp,0)[0] = wp.at<float>(0); 
      cloud.at<cv::Vec3f>(temp,0)[1] = wp.at<float>(1); 
      cloud.at<cv::Vec3f>(temp,0)[2] = wp.at<float>(2); 
      temp++;
    }

    cloudSize[subCnt] = temp;                                   
    if(subCnt==0) 
      _map_pub.PublishMapPoints(cloud);                                        
    else if((cloudSize[subCnt]-cloudSize[subCnt-1])!=0 && subCnt>=1)   
    {                                                                          
      _map_pub.PublishMapPoints(cloud);                                        
    }                                                                          
    subCnt++;                                       
    if(!currFrame->mTcw.empty() && !currFrame->mpRelocalizing)                   
    {  
       _map_pub.PublishCurrentCamera(currFrame->mTcw); 
       _odom_pub.PublishOdom(currFrame->mTcw);
 
    }

}

