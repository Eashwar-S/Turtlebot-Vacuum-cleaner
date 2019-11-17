#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/Odometry.h"
class Walker {
 private:
  ros::NodeHandle nh_;
  bool collision;
  ros::Subscriber laser;
  ros::Subscriber pose;
  ros::Publisher velocity;
  geometry_msgs::Twist msg;
  bool initializeRobotPosition();
  bool collisionDetector(const sensor_msgs::LaserScan::ConstPtr &distance);
  double distanceFinder(const sensor_msgs::LaserScan::ConstPtr &distance);
  int changePath(bool collision);


};
