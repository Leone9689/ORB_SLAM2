#include "ros/ros.h"
#include "miivii_orbslam2_ros/LoadMap.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_map_client");
  if (argc != 2)
  {
    ROS_INFO("usage: load_map_client X ");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<miivii_orbslam2_ros::LoadMap>("/miivii_vslam/load_map");
  miivii_orbslam2_ros::LoadMap srv;
  srv.request.name = argv[1];
  if (client.call(srv))
  {
    ROS_INFO("Load map response: %d", srv.response.status);
  }
  else
  {
    ROS_ERROR("Failed to call service vslam_load_map");
    return 1;
  }

  return 0;
}
