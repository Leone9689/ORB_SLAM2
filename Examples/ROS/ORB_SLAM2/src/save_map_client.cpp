#include "ros/ros.h"
#include "ORB_SLAM2/SaveMap.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_map_client");
  if (argc != 2)
  {
    ROS_INFO("usage: save_map_client X ");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ORB_SLAM2::SaveMap>("save_map");
  ORB_SLAM2::SaveMap srv;
  srv.request.a = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("SaveFlag: %d", (int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service save_map");
    return 1;
  }

  return 0;
}
