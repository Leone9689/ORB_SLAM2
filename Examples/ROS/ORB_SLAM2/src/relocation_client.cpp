#include "ros/ros.h"
#include "ORB_SLAM2/Relocation.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relocation_client");
  if (argc != 2)
  {
    ROS_INFO("usage: relocation_client X ");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ORB_SLAM2::Relocation>("vslam/relocation");
  ORB_SLAM2::Relocation srv;
  srv.request.enable = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("relocation: %d", (bool)srv.response.status);
  }
  else
  {
    ROS_ERROR("Failed to call service relocation");
    return 1;
  }

  return 0;
}
