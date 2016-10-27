#include "ros/ros.h"
#include "beginner_tutorials/SaveMap.h"

bool status(beginner_tutorials::SaveMap::Request  &req,
         beginner_tutorials::SaveMap::Response &res)
{
  res.sum = req.a;
  ROS_INFO("request: bool=%d", (bool)req.a);
  ROS_INFO("sending back response: [%d]", (bool)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_map_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("save_map", status);
  ROS_INFO("Ready to save map.");
  ros::spin();

  return 0;
}
