// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "perimeter_miner/hold_controller.hpp"
#include <algorithm>
#include <cmath>

namespace perimeter_miner
{

void HoldController::setHoldPosition(double x, double y, double heading)
{
  hold_x_ = x;
  hold_y_ = y;
  hold_heading_ = heading;

  // Reset integral terms
  pos_integral_x_ = 0.0;
  pos_integral_y_ = 0.0;
  heading_integral_ = 0.0;
}

MoveCommand HoldController::compute(const RobotState &state, double dt)
{
  MoveCommand cmd;

  // Use provided dt or default to 0.02 (50Hz)
  double real_dt = (dt > 0.0) ? dt : 0.02;

  // Position error
  double err_x = hold_x_ - state.x;
  double err_y = hold_y_ - state.y;

  // Integral terms with anti-windup using real dt
  pos_integral_x_ += err_x * real_dt;
  pos_integral_y_ += err_y * real_dt;
  pos_integral_x_ = std::clamp(pos_integral_x_, -max_pos_integral_, max_pos_integral_);
  pos_integral_y_ = std::clamp(pos_integral_y_, -max_pos_integral_, max_pos_integral_);

  // Heading error
  double err_heading = normalizeAngle(hold_heading_ - state.heading);
  heading_integral_ += err_heading * real_dt;
  heading_integral_ = std::clamp(heading_integral_, -max_heading_integral_,
                                 max_heading_integral_);

  // Proportional control for position
  double pos_cmd_x = pos_kp_ * err_x + pos_ki_ * pos_integral_x_;
  double pos_cmd_y = pos_kp_ * err_y + pos_ki_ * pos_integral_y_;

  // Proportional control for heading using real dt for derivative term
  double heading_cmd = heading_kp_ * err_heading + heading_ki_ * heading_integral_ +
                       heading_kd_ * (err_heading - 0.0) / real_dt;

  // Clamp outputs
  cmd.linear_x = std::clamp(pos_cmd_x, -max_linear_, max_linear_);
  cmd.linear_y = std::clamp(pos_cmd_y, -max_linear_, max_linear_);
  cmd.angular_z = std::clamp(heading_cmd, -max_angular_, max_angular_);

  return cmd;
}

bool HoldController::isAtHoldPosition(const RobotState &state, double tolerance) const
{
  double pos_error = std::hypot(hold_x_ - state.x, hold_y_ - state.y);
  double heading_error = std::abs(normalizeAngle(hold_heading_ - state.heading));

  return pos_error <= tolerance && heading_error < 0.1;  // 0.1 rad ~ 5.7 degrees
}

double HoldController::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double HoldController::angleDiff(double from, double to)
{
  return normalizeAngle(to - from);
}

} // namespace perimeter_miner
