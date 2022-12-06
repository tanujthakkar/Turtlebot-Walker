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
 * @brief Walker source file
 *
 */

#include <turtlebot_walker/walker.hpp>

using std::placeholders::_1;

enum class STATE { INIT, FORWARD, BACK, TURN_RIGHT };

Walker::Walker() : Node("simple_walker"), current_state_{STATE::INIT} {
  this->initialize();
}

void Walker::initialize() {
  vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_cycle_timer_ = this->create_wall_timer(
      100ms, std::bind(&Walker::controlCycleCallback, this));

  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&Walker::scanCallback, this, _1));

  forward_velocity.linear.x = 0.2;
  backward_velocity.linear.x = -0.1;
  turn_velocity.angular.z = -0.2;

  // Start the robot with an angle
  RCLCPP_INFO_STREAM(this->get_logger(), "Start robot with an angle");
  auto turn_left_velocity = turn_velocity;
  turn_left_velocity.angular.z = 0.2;

  rclcpp::Time start_time = this->now();
  double elapsed_second = 0;
  while (elapsed_second < 3) {
    vel_publisher_->publish(turn_left_velocity);
    elapsed_second = (this->now() - start_time).seconds();
  }

  this->goState(STATE::FORWARD);
  RCLCPP_INFO_STREAM(this->get_logger(), "Enter FORWARD state");
}

void Walker::controlCycleCallback() {
  if (last_scan_ == nullptr) {
    return;
  }

  geometry_msgs::msg::Twist output_velocity;

  switch (current_state_) {
    case STATE::FORWARD:
      output_velocity = forward_velocity;

      if (this->forward_to_back()) {
        this->goState(STATE::BACK);
        RCLCPP_INFO_STREAM(this->get_logger(), "Enter BACK state");
      }

      break;

    case STATE::BACK:
      output_velocity = backward_velocity;

      if (this->back_to_turn()) {
        this->goState(STATE::TURN_RIGHT);
        RCLCPP_INFO_STREAM(this->get_logger(), "Enter TURN state");
      }

      break;

    case STATE::TURN_RIGHT:
      output_velocity = turn_velocity;

      if (this->turn_to_forward()) {
        this->goState(STATE::FORWARD);
        RCLCPP_INFO_STREAM(this->get_logger(), "Enter FORWARD state");
      }

      break;

    default:
      break;
  }

  vel_publisher_->publish(output_velocity);
}

void Walker::scanCallback(sensor_msgs::msg::LaserScan::UniquePtr msg) {
  last_scan_ = std::move(msg);
}

void Walker::goState(STATE new_state) {
  this->current_state_ = new_state;
  this->state_start_time_ = this->now();
}

bool Walker::forward_to_back() {
  bool blocked = false;
  // The scan range on both side from center
  int half_view_idxs = 70;  // degree
  int right_check_idx = 360 - half_view_idxs;
  int left_check_idx = 0 + half_view_idxs;
  // Check range on the right half
  for (int i = right_check_idx; i < 360; ++i) {
    if (last_scan_->ranges[i] < this->OBSTACLE_MARGIN) {
      blocked = true;
      break;
    }
  }

  // Check range on the left half
  for (int i = 0; i < left_check_idx; ++i) {
    if (last_scan_->ranges[i] < this->OBSTACLE_MARGIN) {
      blocked = true;
      break;
    }
  }

  return blocked;
}

bool Walker::back_to_turn() {
  auto elapsed = this->now() - this->state_start_time_;
  return elapsed > this->BACK_TIME;
}

bool Walker::turn_to_forward() {
  auto elapsed = this->now() - this->state_start_time_;
  return elapsed > this->TURN_TIME;
}
