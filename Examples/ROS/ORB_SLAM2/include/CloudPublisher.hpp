#ifndef FRAMEPUBLISHER_H
#define FRAMEPUBLISHER_H

#include<opencv2/core/core.hpp>
#include "ros/ros.h"

#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h> 

namespace ROS_ORB_SLAM
{

class CloudPublisher
{
public:
  CloudPublisher();  
  //void refresh();
  //void Update(const cv::Mat cv_ptr);
  void PublishCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);
protected:
  //cv::Mat DrawFrame();
  //void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
  //cv::Mat mIm;
  ros::NodeHandle mNH;
  //ros::Publisher mImagePub;
  ros::Publisher pcl_pub;
  
  sensor_msgs::PointCloud2 output;  
  //int foo;
  //int bar;
};
} // namespace ROS_ORB_SLAM
#endif // FRAMEPUBLISHER_H
