// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 */

/*
 * This code has been adapted and/or modified for use in the 2024-25 WPI Lunabotics MQP.
 * Author: Sam Rooney
 */

#ifndef LUNA_CONTROL__ODOMETRY_HPP_
#define LUNA_CONTROL__ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
#include "rcpputils/rolling_mean_accumulator.hpp"
#else
#include "rcppmath/rolling_mean_accumulator.hpp"
#endif

namespace luna_controller
{
  class Odometry
  {
  public:
    explicit Odometry(size_t velocity_rolling_window_size = 10);

    void init(const rclcpp::Time &time);
    bool update(double left_back_pod_pos, double left_front_pod_pos, double right_back_pod_pos, double right_front_pod_pos, double left_back_wheel_vel, double left_front_wheel_vel, double right_back_wheel_vel, double right_front_wheel_vel, const rclcpp::Time &time);
    void updateOpenLoop(double linear, double angular, double strafe, const rclcpp::Time &time);
    void resetOdometry();

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getHeading() const { return heading_; }
    double getLinear() const { return linear_; }
    double getAngular() const { return angular_; }

    void setWheelParams(double wheel_track, double wheel_base, double wheel_radius);
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
    using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
#else
    using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
#endif

    void integrateRungeKutta2(double linear, double angular, double strafe);
    void integrateExact(double linear, double angular, double strafe);
    void resetAccumulators();

    // Current timestamp:
    rclcpp::Time timestamp_;

    // Current pose:
    double x_;       //   [m]
    double y_;       //   [m]
    double heading_; // [rad]

    // Current velocity:
    double linear_;  //   [m/s]
    double strafe_;  //   [m/s]
    double angular_; // [rad/s]

    // Wheel kinematic parameters [m]:
    double wheel_track_;
    double wheel_base_;
    double wheel_radius_;

    // Previous wheel position/state [rad]:
    double left_back_wheel_old_vel_;
    double left_front_wheel_old_vel_;
    double right_back_wheel_old_vel_;
    double right_front_wheel_old_vel_;
    double left_back_pod_old_pos_;
    double left_front_pod_old_pos_;
    double right_back_pod_old_pos_;
    double right_front_pod_old_pos_;

    // Rolling mean accumulators for the linear and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAccumulator linear_accumulator_;
    RollingMeanAccumulator strafe_accumulator_;
    RollingMeanAccumulator angular_accumulator_;
  };

} // namespace diff_drive_controller

#endif // LUNA_CONTROL__ODOMETRY_HPP_