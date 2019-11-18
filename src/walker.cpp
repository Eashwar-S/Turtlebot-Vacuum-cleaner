#include "turtlebot_vacuum_cleaner/walker.hpp"

Walker::Walker(ros::NodeHandle &node)
: nh_(node) {
  velocity_publisher = nh_.advertise<geometry_msgs::Twist>(
      "/mobile_base/commands/velocity", 100);
  laser_ = nh_.subscribe("scan", 10, &Walker::laserCallback, this);
}
void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr &distance) {
  geometry_msgs::Twist velocity;
  velocity = collisionDetector(distance);
}

geometry_msgs::Twist Walker::collisionDetector(
    const sensor_msgs::LaserScan::ConstPtr &distance) {
  geometry_msgs::Twist velocity;
  float minimumValue = distance->ranges[0];
  ROS_DEBUG_STREAM_ONCE(minimumValue);
  float maximumValue = distance->ranges[0];
  ROS_DEBUG_STREAM_ONCE(maximumValue);
  bool status;
  ROS_INFO_STREAM_ONCE("Finding minimum distance within range");
  auto minimumRange = distance->range_min;
  auto maximumRange = distance->range_min;
  for (auto rangevalues : distance->ranges) {
    if (minimumValue > rangevalues)
      minimumValue = rangevalues;
    if (maximumValue < rangevalues)
      maximumValue = rangevalues;
  }
  if (minimumValue >= maximumRange) {
    if (minimumValue - maximumRange >= 0.3) {
      ROS_INFO_STREAM("Moving forward as no obstacles detected");
      status = moveForward(velocity);
    } else {
      ROS_WARN_STREAM("Possibility of collision detected");
      ROS_INFO_STREAM("Rotating the robot to change direction");
      status = turnLeft(velocity);
    }
  } else if (minimumValue <= minimumRange
      || (minimumValue >= minimumRange && minimumValue <= maximumRange)) {
    ROS_INFO_STREAM(minimumValue);
    ROS_WARN_STREAM("Possibility of collision detected");
    ROS_INFO_STREAM("Rotating the robot to change direction");
    status = turnLeft(velocity);
  }
  return velocity;
}
bool Walker::moveForward(geometry_msgs::Twist velocity) {
  ROS_INFO_STREAM("Moving Forward");
  velocity.linear.x = 0.2;
  velocity.angular.z = 0.0;
  velocity_publisher.publish(velocity);
  return true;
}
bool Walker::turnLeft(geometry_msgs::Twist velocity) {
  ROS_INFO_STREAM("Rotating Left");
  velocity.linear.x = 0;
  velocity.angular.z = 0.1;
  velocity_publisher.publish(velocity);
  return true;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");
  ros::NodeHandle nh_;
  ROS_INFO_STREAM("walker node is initialized");
  Walker walker(nh_);
  ros::spin();
  return 0;
}
Walker::~Walker() {
}
