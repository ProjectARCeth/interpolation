#include "../include/interpolation/bezier.hpp"
#include <iostream>
#include "nav_msgs/Path.h"
#include "ros/ros.h"


int main(int argc, char **argv)
	{
	// Create a node for testing the interpolation things.
	ros::init(argc, argv, "interpolation_test");
	ros::NodeHandle n;

  // Create and set up the publishers.
  ros::Publisher global_path_pub;
  ros::Publisher local_path_pub;
  global_path_pub = n.advertise<nav_msgs::Path>("/global_path", 10);
  local_path_pub = n.advertise<nav_msgs::Path>("/local_path", 10);

  // Create a global path.
  nav_msgs::Path global_path = ###;
  int number_of_path_points = ###;

  // Create a curve object.
  BezierCurve test_curve(global_path, number_of_path_points);

  // Define the curve.
  int center = ###;
  test_curve.setCenterIndex(center);
  test_curve.setCtrlPoints();

  // Rediscretize the path.
  nav_msgs::Path local_path;
  for(float t = 0; t < 1.001; t = t+1)
  {
    Eigen::Vector3d temp_coordinate = test_curve.getCoordinate(t);
    geometry_msgs::PoseStamped temp_pose;
    temp_pose.pose.position.x = temp_coordinate[0];
    temp_pose.pose.position.y = temp_coordinate[1];
    local_path.poses.push_back(temp_pose);
  }

  // Publish to RVIZ.
  while(ros::ok())
  {
    local_path_pub.publish(local_path);
    global_path_pub.publish(global_path);
  }

	return 0;
	}
