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

/**
 * @file walker.hpp
 *
 * @author Eashwar Sathyamurthy
 *
 * @brief declaration for walker class
 *
 * @version 1
 *
 * @date 2019-11-18
 *
 * This .hpp file has declarations for the class attributes and methods for
 * imparting obstacle avoidance functionality in turtlebot.
 *
 */
#ifndef INCLUDE_TURTLEBOT_VACUUM_CLEANER_WALKER_HPP_
#define INCLUDE_TURTLEBOT_VACUUM_CLEANER_WALKER_HPP_

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cstdlib>
#include "ros/ros.h"
#include "ros/console.h"

/**
 *
 * @brief declaration of Walker class
 *
 */

class Walker {
 private:
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh_;
  bool status;
  /// Declaring subscriber object to subscribe scan topic
  ros::Subscriber laser_;
  /// Declaring publisher object to publish cmd_vel topic
  ros::Publisher velocity_publisher;
  /// Declaring object to Twist message to populate linear
  /// and angular velocities.
  geometry_msgs::Twist velocity;
  /**
   * @brief method to rotate turtlebot left
   *
   * @param velocity is an object to access messages from geometry_msgs package.
   *
   * @return boolean - indicates whether rotation was performed or not.
   *
   */
  bool turnLeft(geometry_msgs::Twist velocity);
  /**
   * @brief method to make turtlebot move forward
   *
   * @param velocity is an object to access messages from geometry_msgs package.
   *
   * @return boolean - indicates whether translation was performed or not.
   *
   */
  bool moveForward(geometry_msgs::Twist velocity);

  /**
   * @brief callback method for subscriber of scan topic
   *
   * @param distance is a pointer object to access messages from LaserScan package.
   *
   * @return void
   *
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &distance);
  /**
   * @brief method to detect collision of turtlebot robot
   *
   * @param distance is a pointer object to access messages from LaserScan package.
   *
   * @return velocity - object containing modified rotation and translation components
   *
   */
  geometry_msgs::Twist collisionDetector(
      const sensor_msgs::LaserScan::ConstPtr &distance);

 public:
  /**
   * @brief constructor to initialize publisher and subscriber
   *
   * @param length1 variable for initializing the member linkLength1
   *
   * @param length2 variable for initializing the member linkLength2
   *
   * @return none
   *
   */
  explicit Walker(ros::NodeHandle &node);
  /**
   * @brief destructor for walker class
   */
  ~Walker();
};
#endif  // INCLUDE_TURTLEBOT_VACUUM_CLEANER_WALKER_HPP_
