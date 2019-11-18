/***************************************************************************
 * Copyright (c) 2019, Eashwar Sathyamurthy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "turtlebot_vacuum_cleaner/walker.hpp"
Walker::Walker(ros::NodeHandle &node)
: nh_(node) {
  /// Initializing publisher
  velocity_publisher = nh_.advertise<geometry_msgs::Twist>(
      "/mobile_base/commands/velocity", 100);
  /// Initializing Subscriber
  laser_ = nh_.subscribe("scan", 10, &Walker::laserCallback, this);
}
void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr &distance) {
  geometry_msgs::Twist velocity;
  /// Calling collisionDector method to facilitate unit testing
  velocity = collisionDetector(distance);
}

geometry_msgs::Twist Walker::collisionDetector(
    const sensor_msgs::LaserScan::ConstPtr &distance) {
  geometry_msgs::Twist velocity;
  auto minimumValue = distance->ranges[0];
  ROS_DEBUG_STREAM_ONCE(minimumValue);
  auto maximumValue = distance->ranges[0];
  ROS_DEBUG_STREAM_ONCE(maximumValue);
  bool status;
  ROS_INFO_STREAM_ONCE("Finding minimum distance within range");
  /// Accessing minimum range of the laser
  auto minimumRange = distance->range_min;
  /// Accessing maximum range of the laser
  auto maximumRange = distance->range_min;
  /// Finding minimum and maximum distance of the obstacle
  for (auto rangevalues : distance->ranges) {
    if (minimumValue > rangevalues)
      minimumValue = rangevalues;
    if (maximumValue < rangevalues)
      maximumValue = rangevalues;
  }
  /// Checking for collision
  if (minimumValue >= maximumRange) {
    if (minimumValue - maximumRange >= 0.3) {
      ROS_INFO_STREAM("Moving forward as no obstacles detected");
      /// Moving forward if no obstacle detected
      status = moveForward(velocity);
    } else {
      ROS_WARN_STREAM("Possibility of collision detected");
      ROS_INFO_STREAM("Rotating the robot to change direction");
      /// Changing direction if obstacle detected
      status = turnLeft(velocity);
    }
  } else if (minimumValue <= minimumRange
      || (minimumValue >= minimumRange && minimumValue <= maximumRange)) {
    ROS_INFO_STREAM(minimumValue);
    ROS_WARN_STREAM("Possibility of collision detected");
    ROS_INFO_STREAM("Rotating the robot to change direction");
    /// Changing direction if obstacle detected
    status = turnLeft(velocity);
  }
  return velocity;
}
bool Walker::moveForward(geometry_msgs::Twist velocity) {
  ROS_INFO_STREAM("Moving Forward");
  /// Translating robot with respect to x-axis
  velocity.linear.x = 0.2;
  velocity.angular.z = 0.0;
  /// Publishing updated values
  velocity_publisher.publish(velocity);
  return true;
}
bool Walker::turnLeft(geometry_msgs::Twist velocity) {
  ROS_INFO_STREAM("Rotating Left");
  /// Rotating robot with respect to z-axis
  velocity.linear.x = 0;
  velocity.angular.z = 0.1;
  /// Publishing updated values
  velocity_publisher.publish(velocity);
  return true;
}
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   */
  ros::init(argc, argv, "walker");
  ros::NodeHandle nh_;
  ROS_INFO_STREAM("walker node is initialized");
  Walker walker(nh_);
  ros::spin();
  return 0;
}
Walker::~Walker() {
}
