/******************************************************************************
 * MIT License

Copyright (c) 2022 Tanuj Thakkar
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
* *******************************************************************************
*/

/**
 * @copyright Copyright (c) 2022  Tanuj Thakkar
 * @author Tanuj Thakkar (tanuj@umd.edu)
 * @brief Walker header file
 *
 */

#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::chrono_literals::operator""s;
using std::chrono_literals::operator""ms;

/**
 * @Brief  Walker state
 */
enum class STATE;

/**
 * @Brief  Simple walker behavior that avoid obstacles
 */
class Walker : public rclcpp::Node {
 public:
  /**
   * @Brief  Constructor
   */
  Walker();

 private:
  /**
   * @Brief  Initialize ros functions
   */
  void initialize();

  /**
   * @Brief  Finite state machine for robot behavior
   */
  void controlCycleCallback();

  /**
   * @Brief  Callback function for laser scan subscriber
   *
   * @Param msg Laser scan message
   */
  void scanCallback(sensor_msgs::msg::LaserScan::UniquePtr msg);

  /**
   * @Brief  Transist state to another
   *
   * @Param new_state The next state
   */
  void goState(STATE new_state);

  /**
   * @Brief  Condition from state Forward to Back
   *         If the robot is in range of obstacle avoidance,
   *         this function returns true.
   *
   * @Returns true to transist to Turn
   */
  bool forward_to_back();

  /**
   * @Brief  Condition from state Back to turn
   *         After several seconds in Back state,
   *         this function returns true.
   *
   * @Returns true to transist to Turn
   */
  bool back_to_turn();

  /**
   * @Brief  Condition from state Turn to Forward
   *         After several seconds in Turn state,
   *         this function returns true.
   *
   * @Returns True to transist to Forward
   */
  bool turn_to_forward();

  /**
   * @Brief  Publisher for robot veclocity
   */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

  /**
   * @Brief  Cycle timer for finite state machine
   */
  rclcpp::TimerBase::SharedPtr control_cycle_timer_;

  /**
   * @Brief  Subscriber for laser scan
   */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

  /**
   * @Brief  Robot velocity for Forward state
   */
  geometry_msgs::msg::Twist forward_velocity;

  /**
   * @Brief  Robot velocity for Backward state
   */
  geometry_msgs::msg::Twist backward_velocity;

  /**
   * @Brief  Robot velocity for Turn state
   */
  geometry_msgs::msg::Twist turn_velocity;

  /**
   * @Brief  The latest laser scan message
   */
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

  /**
   * @Brief  Current robot state
   */
  STATE current_state_;

  /**
   * @Brief  Start time for each state
   *         Used for measuring time spent in a state
   */
  rclcpp::Time state_start_time_;

  /**
   * @Brief  Duration in Back state
   */
  const rclcpp::Duration BACK_TIME{1s};

  /**
   * @Brief  Duration in Turn state
   */
  const rclcpp::Duration TURN_TIME{4s};

  /**
   * @Brief  The safety margin to obstacle
   */
  const float OBSTACLE_MARGIN{0.3};
};
