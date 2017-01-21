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

#include <thread>
//#include <pangolin/pangolin.h>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h> 

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "System.h"
#include "PangolinViewer.h"
#include "Viewer.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "Map.h"
#include "MapPublisher.hpp"
#include "OdomPublisher.hpp"
#include "miivii_orbslam2_ros/CreateMap.h"
#include "miivii_orbslam2_ros/LoadMap.h"
#include "miivii_orbslam2_ros/SaveMap.h"
#include "miivii_orbslam2_ros/Relocation.h"
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::CameraInfo> sync_pol;

class ImageGrabber
{
public:
ImageGrabber(ORB_SLAM2::System* pSLAM)
            :cloudCnt(0),keyFrameCnt(0),relocate(false),
             distance(0),mapDensity(0),keyFrameDensity(0),S(0),createMap(false),mapName("none"),mpSLAM(pSLAM),
             rgb_sub(n, "/miivii_vslam/rgb/image_raw", 1),
             depth_sub(n, "/miivii_vslam/depth_registered/image_raw", 1),
             info_sub(n, "/miivii_vslam/depth/camera_info", 1),
             sync(sync_pol(10), rgb_sub,depth_sub,info_sub)
    {
      depth_pub = n.advertise<sensor_msgs::Image>("/miivii_vslam/depth_scan",1);
      camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/miivii_vslam/camera_info",1);
      map_info_pub = n.advertise<std_msgs::Float32MultiArray>("/miivii_vslam/map_info", 10);
      if(!relocate && !createMap) 
      {
        rgb_sub.unsubscribe();
        depth_sub.unsubscribe();
        info_sub.unsubscribe();
      }
      sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,this,_1,_2,_3));

    }
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const sensor_msgs::CameraInfoConstPtr& msgInfo);
    bool CreateMapStatus(miivii_orbslam2_ros::CreateMap::Request &req,miivii_orbslam2_ros::CreateMap::Response &res);            
    bool SaveMapStatus(miivii_orbslam2_ros::SaveMap::Request &req,miivii_orbslam2_ros::SaveMap::Response &res);
    bool RelocationStatus(miivii_orbslam2_ros::Relocation::Request &req,miivii_orbslam2_ros::Relocation::Response &res);                           
    bool LoadMapStatus(miivii_orbslam2_ros::SaveMap::Request &req,miivii_orbslam2_ros::SaveMap::Response &res); 
    void square(vector<ORB_SLAM2::MapPoint*> mapPoints,int n);
    void quarter(vector<ORB_SLAM2::MapPoint*> mapPoints,float x_min,float x_max,float y_min,float y_max,int n);
    float GetMapSquare(vector<ORB_SLAM2::MapPoint*> mapPoints);
public:
    
    int cloudCnt;                              
    int keyFrameCnt;                              
    bool relocate;
    double distance;
    double mapDensity;
    double keyFrameDensity;
    float S;
    bool createMap;
    string mapName;
    std::map<int,int> cloudSize;             
    std::map<int,cv::Mat> translation;             
    ros::NodeHandle n;                       
    cv::Mat cloud;    
    ORB_SLAM2::System* mpSLAM;               
    ORB_SLAM2::Frame* currFrame;             
    
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
    message_filters::Synchronizer<sync_pol> sync;
    
    ros::Publisher depth_pub;
    ros::Publisher camera_info_pub;
    ros::Publisher map_info_pub;
    ORB_SLAM2::MapPublisher _map_pub;
    ORB_SLAM2::OdomPublisher _odom_pub;
    
};

bool ImageGrabber::CreateMapStatus(miivii_orbslam2_ros::CreateMap::Request  &req,  
                                   miivii_orbslam2_ros::CreateMap::Response &res)            
{                                                                          
  res.status = req.enable;                                                   
  ROS_INFO("CreateMap response:%d", (bool)res.status);               
  createMap = res.status;                               
  mpSLAM->Initialize(createMap,mapName);
  
  if(createMap) {
    rgb_sub.subscribe();
    depth_sub.subscribe();
    info_sub.subscribe();
  }else{
    rgb_sub.unsubscribe();
    depth_sub.unsubscribe();
    info_sub.unsubscribe();
  }  
  return true;                                                             
}                                                                          
bool ImageGrabber::LoadMapStatus(miivii_orbslam2_ros::SaveMap::Request  &req, 
                                 miivii_orbslam2_ros::SaveMap::Response &res) 
{                                                            
  res.status = true;                                           
  ROS_INFO("LoadMap response:%s", req.name.c_str());    
  mapName = req.name;
  mpSLAM->Initialize(createMap,mapName+".map");
  return true;                                               
}                           
bool ImageGrabber::SaveMapStatus(miivii_orbslam2_ros::SaveMap::Request  &req, 
                                 miivii_orbslam2_ros::SaveMap::Response &res) 
{                                                            
  res.status = true;                                           
  ROS_INFO("SaveMap response:%s", req.name.c_str());    
  mpSLAM->SaveMap(req.name+".map");                                

  return true;                                               
}                                                            
bool ImageGrabber::RelocationStatus(miivii_orbslam2_ros::Relocation::Request  &req,  
                                    miivii_orbslam2_ros::Relocation::Response &res)            
{                                                                          
   res.status = req.enable;                                                   
   //ROS_INFO("request: bool=%d", (bool)req.flag);                            
   ROS_INFO("Relocation response:%d", (bool)res.status);               
   relocate = res.status;                               
   if(relocate) {
     rgb_sub.subscribe();
     depth_sub.subscribe();
     info_sub.subscribe();
   }else{
     rgb_sub.unsubscribe();
     depth_sub.unsubscribe();
     info_sub.unsubscribe();
   }  
   return true;                                                             
}                                                                          

void ImageGrabber::square(vector<ORB_SLAM2::MapPoint*> mapPoints,int n)
{
    float x_min=9999,x_max=-9999;
    float y_min=9999,y_max=-9999;
    for(auto mp: mapPoints)    
    {
      cv::Mat wp = mp->GetWorldPos();  
      if(x_min > wp.at<float>(2))   
        x_min = wp.at<float>(2);
      if(x_max < wp.at<float>(2))   
        x_max = wp.at<float>(2);

      if(y_min > wp.at<float>(0))   
        y_min = wp.at<float>(0);
      if(y_max < wp.at<float>(0))   
        y_max = wp.at<float>(0);
    }
    
    if(n>1)
      quarter(mapPoints,x_min,x_max,y_min,y_max,n/4); 
    else
      S = S + (x_max-x_min)*(y_max-y_min);
}
void ImageGrabber::quarter(vector<ORB_SLAM2::MapPoint*> mapPoints,float x_min,float x_max,float y_min,float y_max,int n)
{
    float x_mid=0,y_mid=0;
    
    float x0_min=9999,x0_max=-9999,y0_min=9999,y0_max=-9999;
    float x1_min=9999,x1_max=-9999,y1_min=9999,y1_max=-9999;
    float x2_min=9999,x2_max=-9999,y2_min=9999,y2_max=-9999;
    float x3_min=9999,x3_max=-9999,y3_min=9999,y3_max=-9999;
    
    x_mid = (x_max + x_min)/2;
    y_mid = (y_max + y_min)/2;
    for(auto mp: mapPoints)    
    {
      cv::Mat wp = mp->GetWorldPos();  
      if(x_min<wp.at<float>(2) && wp.at<float>(2)<x_mid && y_mid<wp.at<float>(0) && wp.at<float>(0)<y_max )//0
      {
        if(x0_min > wp.at<float>(2)) 
          x0_min = wp.at<float>(2);  
        if(x0_max < wp.at<float>(2)) 
          x0_max = wp.at<float>(2);  
        if(y0_min > wp.at<float>(0)) 
          y0_min = wp.at<float>(0);  
        if(y0_max < wp.at<float>(0)) 
          y0_max = wp.at<float>(0); 
      }
      if(x_mid<wp.at<float>(2) && wp.at<float>(2)<x_max && y_mid<wp.at<float>(0) && wp.at<float>(0)<y_max )//1
      {
        if(x1_min > wp.at<float>(2)) 
          x1_min = wp.at<float>(2);  
        if(x1_max < wp.at<float>(2)) 
          x1_max = wp.at<float>(2);  
        if(y1_min > wp.at<float>(0)) 
          y1_min = wp.at<float>(0);  
        if(y1_max < wp.at<float>(0)) 
          y1_max = wp.at<float>(0); 
      }
      if(x_min<wp.at<float>(2) && wp.at<float>(2)<x_mid && y_min<wp.at<float>(0) && wp.at<float>(0)<y_mid )//2
      {
        if(x2_min > wp.at<float>(2)) 
          x2_min = wp.at<float>(2);  
        if(x2_max < wp.at<float>(2)) 
          x2_max = wp.at<float>(2);  
        if(y2_min > wp.at<float>(0)) 
          y2_min = wp.at<float>(0);  
        if(y2_max < wp.at<float>(0)) 
          y2_max = wp.at<float>(0); 
      }
      if(x_mid<wp.at<float>(2) && wp.at<float>(2)<x_max && y_min<wp.at<float>(0) && wp.at<float>(0)<y_mid )//3
      {
        if(x3_min > wp.at<float>(2)) 
          x3_min = wp.at<float>(2);  
        if(x3_max < wp.at<float>(2)) 
          x3_max = wp.at<float>(2);  
        if(y3_min > wp.at<float>(0)) 
          y3_min = wp.at<float>(0);  
        if(y3_max < wp.at<float>(0)) 
          y3_max = wp.at<float>(0); 
      }
    }   
    if(x0_min==9999||x0_max==-9999||y0_min==9999||y0_max==-9999)
      x0_min = x0_max = y0_min = y0_max = 0;
    if(x1_min==9999||x1_max==-9999||y1_min==9999||y1_max==-9999)
      x1_min = x1_max = y1_min = y1_max = 0;
    if(x2_min==9999||x2_max==-9999||y2_min==9999||y2_max==-9999)
      x2_min = x2_max = y2_min = y2_max = 0;
    if(x3_min==9999||x3_max==-9999||y3_min==9999||y3_max==-9999)
      x3_min = x3_max = y3_min = y3_max = 0;
    
    if(n>1)
    {
      quarter(mapPoints,x_min,x_mid,y_mid,y_max,n/4); 
      quarter(mapPoints,x_mid,x_max,y_mid,y_max,n/4); 
      quarter(mapPoints,x_min,x_mid,y_min,y_mid,n/4); 
      quarter(mapPoints,x_mid,x_max,y_min,y_mid,n/4); 
    }
    else
      S = S+ (x0_max-x0_min)*(y0_max-y0_min)+(x1_max-x1_min)*(y1_max-y1_min)+(x2_max-x2_min)*(y2_max-y2_min)+(x3_max-x3_min)*(y3_max-y3_min); 
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

    ros::ServiceServer create_map_service = nh.advertiseService("/miivii_vslam/create_map", &ImageGrabber::CreateMapStatus,&igb);
    ros::ServiceServer load_map_service = nh.advertiseService("/miivii_vslam/load_map", &ImageGrabber::LoadMapStatus,&igb);          
    ros::ServiceServer save_map_service = nh.advertiseService("/miivii_vslam/save_map", &ImageGrabber::SaveMapStatus,&igb);          
    ros::ServiceServer relocate_service = nh.advertiseService("/miivii_vslam/relocation", &ImageGrabber::RelocationStatus,&igb);
	  
    //igb.SetupStatus();
     
    /*message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    */
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

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const sensor_msgs::CameraInfoConstPtr& msgInfo)
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
    int mapPoints = mpSLAM->GetMap()->GetAllMapPoints().size();
    cloud = cv::Mat::zeros(mapPoints, 1, CV_32FC3 ); 
    
    int temp=0;
 
    for(auto mp: mpSLAM->GetMap()->GetAllMapPoints())    
    {
      cv::Mat wp = mp->GetWorldPos();  
      
      cloud.at<cv::Vec3f>(temp,0)[0] = wp.at<float>(0); 
      cloud.at<cv::Vec3f>(temp,0)[1] = wp.at<float>(1); 
      cloud.at<cv::Vec3f>(temp,0)[2] = wp.at<float>(2); 
      temp++;
    
    }

    cloudSize[cloudCnt] = temp;                                   
    if(cloudCnt==0) 
      _map_pub.PublishMapPoints(cloud);                                        
    else if((cloudSize[cloudCnt]-cloudSize[cloudCnt-1])!=0 && cloudCnt>=1)   
    {                                                                          
      _map_pub.PublishMapPoints(cloud);                                        
    }                                                                          
    cloudCnt++;                                       
    
    static tf::TransformBroadcaster laser_broadcaster;
    tf::Transform laser_transform;                    

    if(!currFrame->mTcw.empty()) 
    {
      
      if(!currFrame->mpRelocalizing)                   
      {  
        sensor_msgs::Image imageD = *msgD;                                                                           
        //imageD.header.stamp = msgInfo->header.stamp;
        //imageD.header.stamp = ros::Time::now();
        sensor_msgs::CameraInfo depthInfo = *msgInfo;
        //depthInfo.header.stamp = msgInfo->header.stamp;
        //depthInfo.header.stamp = ros::Time::now();
        //imageD.header.frame_id = "/camera_depth_frame";
        
        //imageD.header.frame_id = "/camera_depth_frame";  
        //imageD.header.child_frame_id = "/camera_depth_frame";  
        
        //depthInfo.header.frame_id = "/camera_depth_frame";  
        //depthInfo.header.child_frame_id = "/camera_depth_frame";  
        
        _map_pub.PublishCurrentCamera(currFrame->mTcw); 
        depth_pub.publish(imageD); 
        camera_info_pub.publish(depthInfo);
        _odom_pub.PublishOdom(currFrame->mTcw);
        
        cv::Mat Twc = (currFrame->mTcw).inv();                                                                       
        Eigen::Matrix<double,3,3> eigMat =ORB_SLAM2::Converter::toMatrix3d(Twc);                                     
        Eigen::Vector3d ea = eigMat.eulerAngles(0, 1, 2);                                                            
        cv::Mat tcwMat = Twc.rowRange(0,3).col(3);                                                                   
                                                                                                                     
        //imageD.header.stamp = msgInfo->header.stamp;                                                                 
        _map_pub.PublishCurrentCamera(currFrame->mTcw);                                                              
        depth_pub.publish(imageD);                                                                                   
        camera_info_pub.publish(msgInfo);                                                                            
                                                                                                                     
        laser_transform.setOrigin( tf::Vector3(tcwMat.at<float>(2), -tcwMat.at<float>(0),-tcwMat.at<float>(1)) );    
        
        Eigen::Quaterniond qMat(eigMat);                     
        tf::Quaternion q(qMat.z(),-qMat.x(),-qMat.y(),qMat.w());        
        //q.setRPY(-ea[2],ea[0],-ea[1]);                                                                               
        laser_transform.setRotation(q);                                                                              
        laser_broadcaster.sendTransform(tf::StampedTransform(laser_transform, imageD.header.stamp, "map", "odom"));  
                                                                                                                    
        tf::Transform laser_transform;                    
         
      } 
    
      if(currFrame->keyFrame)
      {
        const unsigned int data_sz = 4;
        std_msgs::Float32MultiArray m;

        m.layout.dim.push_back(std_msgs::MultiArrayDimension());
        m.layout.dim[0].size = data_sz;
        m.layout.dim[0].stride = 1;
        m.layout.dim[0].label = "map_info";
        
        square(mpSLAM->GetMap()->GetAllMapPoints(),64);
        translation[keyFrameCnt] = currFrame->mTcw; 
        if(keyFrameCnt >0)
        {
          cv::Mat Tcr = translation[keyFrameCnt] * translation[keyFrameCnt-1].inv();
          cv::Mat tcwMat = Tcr.rowRange(0,3).col(3); 
          distance += fabs(cv::norm(tcwMat));
          mapDensity = mapPoints/S;
        }
        keyFrameCnt++;
        keyFrameDensity=keyFrameCnt/S;
        
        m.data.resize(data_sz);
        m.data[0] = mapDensity;
        m.data[1] = keyFrameDensity;
        m.data[2] = S;
        m.data[3] = float(!currFrame->mpRelocalizing);//tracking 1   relocation 0

        map_info_pub.publish(m);
        S =0 ;
      } 
    }

} 
