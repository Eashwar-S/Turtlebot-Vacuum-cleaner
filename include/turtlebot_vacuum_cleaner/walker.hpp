#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/Odometry.h"
#include <cstdlib>
#include "ros/console.h"

class Walker {
 private:
  ros::NodeHandle nh_;
  ros::Subscriber laser_;
  ros::Publisher velocity_publisher;
  geometry_msgs::Twist velocity;
  bool turnLeft(geometry_msgs::Twist velocity);
  bool turnRight(geometry_msgs::Twist velocity);
  bool moveForward(geometry_msgs::Twist velocity);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &distance);
  geometry_msgs::Twist collisionDetector(
      const sensor_msgs::LaserScan::ConstPtr &distance);
 public:
  explicit Walker(ros::NodeHandle &node);
  ~Walker();
};
