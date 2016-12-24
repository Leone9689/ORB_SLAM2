#include "CloudPublisher.hpp"
#include <sstream>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

namespace ROS_ORB_SLAM
{
  CloudPublisher::CloudPublisher()
{
  //output.header.frame_id = "/world";  
  pcl_pub = mNH.advertise<sensor_msgs::PointCloud2> ("/ORB_SLAM2/Cloud", 10,true);
}

void CloudPublisher::PublishCloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
{

    //output = cloud;
    pcl::toROSMsg(cloud, output);  
    output.header.stamp = ros::Time::now();
    pcl_pub.publish(output); 
}

} // namespace ROS_ORB_SLAM
