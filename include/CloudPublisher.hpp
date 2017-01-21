#ifndef CLOUDPUBLISHER_H
#define CLOUDPUBLISHER_H

#include<opencv2/core/core.hpp>
#include "ros/ros.h"

#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h> 

namespace ORB_SLAM2
{

class CloudPublisher
{
public:
  CloudPublisher();  
  void PublishCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);
protected:
  ros::NodeHandle mNH;
  ros::Publisher pcl_pub;
  
  sensor_msgs::PointCloud2 output;  
};
} // namespace ROS_ORB_SLAM
#endif // FRAMEPUBLISHER_H
