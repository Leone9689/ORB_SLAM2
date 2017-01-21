#include "ros/ros.h"
#include "miivii_orbslam2_ros/SaveMap.h"
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
  ros::ServiceClient client = n.serviceClient<miivii_orbslam2_ros::SaveMap>("/miivii_vslam/save_map");
  miivii_orbslam2_ros::SaveMap srv;
  srv.request.name = argv[1];
  if (client.call(srv))
  {
    ROS_INFO("Save map response: %d", srv.response.status);
  }
  else
  {
    ROS_ERROR("Failed to call service vslam_save_map");
    return 1;
  }

  return 0;
}
