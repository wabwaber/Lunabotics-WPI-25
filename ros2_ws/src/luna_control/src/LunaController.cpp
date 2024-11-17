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
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

/*
 * This code has been adapted and/or modified for use in the 2024-25 WPI Lunabotics MQP.
 * Author: Sam Rooney
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include "luna_control/LunaController.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace luna_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

LunaController::LunaController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn LunaController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration LunaController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.left_back_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.left_front_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.right_back_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.right_front_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  
  for (const auto & joint_name : params_.left_back_pod_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : params_.left_front_pod_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : params_.right_back_pod_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : params_.right_front_pod_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration LunaController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  
  for (const auto & joint_name : params_.left_back_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.left_front_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.right_back_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.right_front_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  
  for (const auto & joint_name : params_.left_back_pod_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : params_.left_front_pod_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : params_.right_back_pod_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : params_.right_front_pod_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type LunaController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  if (get_lifecycle_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  Twist command = *last_command_msg;
  double & strafe_command = command.twist.linear.y;
  double & linear_command = command.twist.linear.x;
  double & angular_command = command.twist.angular.z;

  previous_update_timestamp_ = time;

  // Apply (possibly new) multipliers:
  const double wheel_track = params_.wheel_track;
  const double wheel_radius = params_.wheel_radius;
  const double wheel_base = params_.wheel_base;

  if (params_.open_loop)
  {
    odometry_.updateOpenLoop(linear_command, angular_command, time);
  }
  else
  {
    double left_back_wheel_feedback_mean = 0.0;
    double left_front_wheel_feedback_mean = 0.0;
    double right_back_wheel_feedback_mean = 0.0;
    double right_front_wheel_feedback_mean = 0.0;

    double left_back_pod_feedback_mean = 0.0;
    double left_front_pod_feedback_mean = 0.0;
    double right_back_pod_feedback_mean = 0.0;
    double right_front_pod_feedback_mean = 0.0;
    for (size_t index = 0; index < static_cast<size_t>(wheels_per_side_); ++index)
    {
      const double left_back_wheel_feedback = registered_left_back_wheel_handles_[index].feedback.get().get_value();
      const double left_front_wheel_feedback = registered_left_front_wheel_handles_[index].feedback.get().get_value();
      const double right_back_wheel_feedback = registered_right_back_wheel_handles_[index].feedback.get().get_value();
      const double right_front_wheel_feedback = registered_right_front_wheel_handles_[index].feedback.get().get_value();

      if (std::isnan(left_back_wheel_feedback) || std::isnan(left_front_wheel_feedback) || std::isnan(right_back_wheel_feedback) || std::isnan(right_front_wheel_feedback))
      {
        RCLCPP_ERROR(
          logger, "Either the left or right wheel %s is invalid for index [%zu]", HW_IF_VELOCITY,
          index);
        return controller_interface::return_type::ERROR;
      }

      const double left_back_pod_feedback = registered_left_back_pod_handles_[index].feedback.get().get_value();
      const double left_front_pod_feedback = registered_left_front_pod_handles_[index].feedback.get().get_value();
      const double right_back_pod_feedback = registered_right_back_pod_handles_[index].feedback.get().get_value();
      const double right_front_pod_feedback = registered_right_front_pod_handles_[index].feedback.get().get_value();

      if (std::isnan(left_back_pod_feedback) || std::isnan(left_front_pod_feedback) || std::isnan(right_back_pod_feedback) || std::isnan(right_front_pod_feedback))
      {
        RCLCPP_ERROR(
          logger, "Either the left or right pod %s is invalid for index [%zu]", HW_IF_POSITION,
          index);
        return controller_interface::return_type::ERROR;
      }

      left_back_wheel_feedback_mean += left_back_wheel_feedback;
      left_front_wheel_feedback_mean += left_front_wheel_feedback;
      right_back_wheel_feedback_mean += right_back_wheel_feedback;
      right_front_wheel_feedback_mean += right_front_wheel_feedback;
      
      left_back_pod_feedback_mean += left_back_pod_feedback;
      left_front_pod_feedback_mean += left_front_pod_feedback;
      right_back_pod_feedback_mean += right_back_pod_feedback;
      right_front_pod_feedback_mean += right_front_pod_feedback;
    }

    left_back_wheel_feedback_mean /= static_cast<double>(wheels_per_side_);
    left_front_wheel_feedback_mean /= static_cast<double>(wheels_per_side_);
    right_back_wheel_feedback_mean /= static_cast<double>(wheels_per_side_);
    right_front_wheel_feedback_mean /= static_cast<double>(wheels_per_side_);
    left_back_pod_feedback_mean /= static_cast<double>(wheels_per_side_);
    left_front_pod_feedback_mean /= static_cast<double>(wheels_per_side_);
    right_back_pod_feedback_mean /= static_cast<double>(wheels_per_side_);
    right_front_pod_feedback_mean /= static_cast<double>(wheels_per_side_);

    // TODO: update odometry functions when I update the odometry class
    if (params_.position_feedback)
    {
      odometry_.update(left_feedback_mean, right_feedback_mean, time);
    }
    else
    {
      odometry_.updateFromVelocity(
        left_feedback_mean * wheel_radius * period.seconds(),
        right_feedback_mean * wheel_radius * period.seconds(), time);
    }
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  bool should_publish = false;
  try
  {
    if (previous_publish_timestamp_ + publish_period_ < time)
    {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  }
  catch (const std::runtime_error &)
  {
    // Handle exceptions when the time source changes and initialize publish timestamp
    previous_publish_timestamp_ = time;
    should_publish = true;
  }

  if (should_publish)
  {
    if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinear();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  auto & last_command = previous_commands_.back().twist;
  auto & second_to_last_command = previous_commands_.front().twist;
  limiter_linear_.limit(
    linear_command, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
  limiter_strafe_.limit(
    strafe_command, last_command.linear.y, second_to_last_command.linear.y, period.seconds());
  limiter_angular_.limit(
    angular_command, last_command.angular.z, second_to_last_command.angular.z, period.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(command);

  //    Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = time;
    limited_velocity_command.twist = command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // compute target wheel velocities, pod positions
  double left_back_wheel_velocity = 0.0;
  double left_front_wheel_velocity = 0.0;
  double right_back_wheel_velocity = 0.0;
  double right_front_wheel_velocity = 0.0;

  double left_back_pod_position = 0.0;
  double left_front_pod_position = 0.0;
  double right_back_pod_position = 0.0;
  double right_front_pod_position = 0.0;

  // TODO: check all of this, i guarantee i screwed up signs
  if (angular_command == 0.0)
  {
    left_back_wheel_velocity = linear_command;
    left_front_wheel_velocity = linear_command;
    right_back_wheel_velocity = linear_command;
    right_front_wheel_velocity = linear_command;
    left_back_pod_position = 0.0;
    left_front_pod_position = 0.0;
    right_back_pod_position = 0.0;
    right_front_pod_position = 0.0;
  }
  else if (linear_command == 0.0)
  {
    const double theta_L = (M_PI / 2.0) - atan(wheel_track / wheel_base);
    const double theta_R = (M_PI / 2.0) + atan(wheel_track / wheel_base);
    const double R = sqrt(pow(wheel_track/2, 2) + pow(wheel_base/2, 2));
    const double v = angular_command * R;
    left_back_wheel_velocity = -v;
    left_front_wheel_velocity = -v;
    right_back_wheel_velocity = v;
    right_front_wheel_velocity = v;
    left_back_pod_position = -theta_L;
    left_front_pod_position = theta_L;
    right_back_pod_position = -theta_R;
    right_front_pod_position = theta_R;
  }
  else
  {
    const double icc = linear_command / angular_command; // y component?
    const double theta_L = (M_PI / 2.0) - atan(((2*icc) + wheel_track) / wheel_base);
    const double theta_R = (M_PI / 2.0) - atan(((2*icc) - wheel_track) / wheel_base);
    const double R_L = wheel_base / (2 * cos((M_PI/2) - theta_L));
    const double R_R = wheel_base / (2 * cos((M_PI/2) - theta_R));
    const double v_L = linear_command * (R_L / icc);
    const double v_R = linear_command * (R_L / icc);

    left_back_wheel_velocity = v_L;
    left_front_wheel_velocity = v_L;
    right_back_wheel_velocity = v_R;
    right_front_wheel_velocity = v_R;
    left_back_pod_position = -theta_L;
    left_front_pod_position = theta_L;
    right_back_pod_position = -theta_R;
    right_front_pod_position = theta_R;
  }

  // Set wheels velocities
  // TODO: set wheel velocities to zero unless pod position is close enough to target
  for (size_t index = 0; index < static_cast<size_t>(wheels_per_side_); ++index)
  {
    registered_left_back_wheel_handles_[index].velocity.get().set_value(left_back_wheel_velocity);
    registered_left_front_wheel_handles_[index].velocity.get().set_value(left_front_wheel_velocity);
    registered_right_back_wheel_handles_[index].velocity.get().set_value(right_back_wheel_velocity);
    registered_right_front_wheel_handles_[index].velocity.get().set_value(right_front_wheel_velocity);

    registered_left_back_pod_handles_[index].position.get().set_value(left_back_pod_position);
    registered_left_front_pod_handles_[index].position.get().set_value(left_front_pod_position);
    registered_right_back_pod_handles_[index].position.get().set_value(right_back_pod_position);
    registered_right_front_pod_handles_[index].position.get().set_value(right_front_pod_position);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn LunaController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  if (params_.left_back_wheel_names.size() != params_.right_back_wheel_names.size() || params_.left_front_wheel_names.size() != params_.right_front_wheel_names.size() || params_.left_back_wheel_names.size() != params_.right_front_wheel_names.size())
  {
    RCLCPP_ERROR(
      logger, "The number of wheels lb [%zu], lf [%zu], rb [%zu], and rf [%zu] are different",
      params_.left_back_wheel_names.size(), params_.left_front_wheel_names.size(), params_.right_back_wheel_names.size(), params_.right_front_wheel_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  const double wheel_track = params_.wheel_track;
  const double wheel_base = params_.wheel_base;
  const double wheel_radius = params_.wheel_radius;

  // TODO: update odometry functions when i update the odometry class
  odometry_.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
  odometry_.setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

  cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};
  publish_limited_velocity_ = params_.publish_limited_velocity;

  limiter_linear_ = SpeedLimiter(
    params_.linear.x.has_velocity_limits, params_.linear.x.has_acceleration_limits,
    params_.linear.x.has_jerk_limits, params_.linear.x.min_velocity, params_.linear.x.max_velocity,
    params_.linear.x.min_acceleration, params_.linear.x.max_acceleration, params_.linear.x.min_jerk,
    params_.linear.x.max_jerk);

  limiter_strafe_ = SpeedLimiter(
    params_.linear.y.has_velocity_limits, params_.linear.y.has_acceleration_limits,
    params_.linear.y.has_jerk_limits, params_.linear.y.min_velocity, params_.linear.y.max_velocity,
    params_.linear.y.min_acceleration, params_.linear.y.max_acceleration, params_.linear.y.min_jerk,
    params_.linear.y.max_jerk);

  limiter_angular_ = SpeedLimiter(
    params_.angular.z.has_velocity_limits, params_.angular.z.has_acceleration_limits,
    params_.angular.z.has_jerk_limits, params_.angular.z.min_velocity,
    params_.angular.z.max_velocity, params_.angular.z.min_acceleration,
    params_.angular.z.max_acceleration, params_.angular.z.min_jerk, params_.angular.z.max_jerk);

  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // left and right sides are both equal at this point
  wheels_per_side_ = static_cast<int>(params_.left_back_wheel_names.size());

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ =
      get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
    DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<Twist> msg) -> void
    {
      if (!subscriber_is_active_)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
        return;
      }
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
      {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received TwistStamped with zero timestamp, setting it to current "
          "time, this message will only be shown once");
        msg->header.stamp = get_node()->get_clock()->now();
      }
      received_velocity_msg_ptr_.set(std::move(msg));
    });

  // initialize odometry publisher and message
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  // Append the tf prefix if there is one
  std::string tf_prefix = "";
  if (params_.tf_frame_prefix_enable)
  {
    if (params_.tf_frame_prefix != "")
    {
      tf_prefix = params_.tf_frame_prefix;
    }
    else
    {
      tf_prefix = std::string(get_node()->get_namespace());
    }

    if (tf_prefix == "/")
    {
      tf_prefix = "";
    }
    else
    {
      tf_prefix = tf_prefix + "/";
    }
  }

  const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
  const auto base_frame_id = tf_prefix + params_.base_frame_id;

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_frame_id;
  odometry_message.child_frame_id = base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LunaController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto left_back_wheel_result =
    configure_wheel_segment("left_back", params_.left_back_wheel_names, registered_left_back_wheel_handles_);
  const auto left_front_wheel_result =
    configure_wheel_segment("left_front", params_.left_front_wheel_names, registered_left_front_wheel_handles_);
  const auto right_back_wheel_result =
    configure_wheel_segment("right_back", params_.right_back_wheel_names, registered_right_back_wheel_handles_);
  const auto right_front_wheel_result =
    configure_wheel_segment("right_front", params_.right_front_wheel_names, registered_right_front_wheel_handles_);

  if (
    left_back_wheel_result == controller_interface::CallbackReturn::ERROR ||
    left_front_wheel_result == controller_interface::CallbackReturn::ERROR ||
    right_back_wheel_result == controller_interface::CallbackReturn::ERROR ||
    right_front_wheel_result == controller_interface::CallbackReturn::ERROR)

  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (registered_left_back_wheel_handles_.empty() || registered_left_front_wheel_handles_.empty() || registered_right_back_wheel_handles_.empty() || registered_right_front_wheel_handles_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "One or more of the wheel interfaces are non existent");
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto left_back_pod_result =
    configure_pod_segment("left_back", params_.left_back_pod_names, registered_left_back_pod_handles_);
  const auto left_front_pod_result =
    configure_pod_segment("left_front", params_.left_front_pod_names, registered_left_front_pod_handles_);
  const auto right_back_pod_result =
    configure_pod_segment("right_back", params_.right_back_pod_names, registered_right_back_pod_handles_);
  const auto right_front_pod_result =
    configure_pod_segment("right_front", params_.right_front_pod_names, registered_right_front_pod_handles_);

  if (
    left_back_wheel_result == controller_interface::CallbackReturn::ERROR ||
    left_front_wheel_result == controller_interface::CallbackReturn::ERROR ||
    right_back_wheel_result == controller_interface::CallbackReturn::ERROR ||
    right_front_wheel_result == controller_interface::CallbackReturn::ERROR)

  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (registered_left_back_pod_handles_.empty() || registered_left_front_pod_handles_.empty() || registered_right_back_pod_handles_.empty() || registered_right_front_pod_handles_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "One or more of the pod interfaces are non existent");
    return controller_interface::CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscribers and publishers are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LunaController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  if (!is_halted)
  {
    halt();
    is_halted = true;
  }
  registered_left_back_wheel_handles_.clear();
  registered_left_front_wheel_handles_.clear();
  registered_right_back_wheel_handles_.clear();
  registered_right_front_wheel_handles_.clear();
  registered_left_back_pod_handles_.clear();
  registered_left_front_pod_handles_.clear();
  registered_right_back_pod_handles_.clear();
  registered_right_front_pod_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LunaController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LunaController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool LunaController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  registered_left_back_wheel_handles_.clear();
  registered_left_front_wheel_handles_.clear();
  registered_right_back_wheel_handles_.clear();
  registered_right_front_wheel_handles_.clear();
  registered_left_back_pod_handles_.clear();
  registered_left_front_pod_handles_.clear();
  registered_right_back_pod_handles_.clear();
  registered_right_front_pod_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

controller_interface::CallbackReturn LunaController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void LunaController::halt()
{
  const auto halt_wheels = [](auto & wheel_handles)
  {
    for (const auto & wheel_handle : wheel_handles)
    {
      wheel_handle.velocity.get().set_value(0.0);
    }
  };
  const auto halt_pods = [](auto & pod_handles)
  {
    for (const auto & pod_handle : pod_handles)
    {
      pod_handle.position.get().set_value(0.0);
    }
  };

  halt_wheels(registered_left_back_wheel_handles_);
  halt_wheels(registered_left_front_wheel_handles_);
  halt_wheels(registered_right_back_wheel_handles_);
  halt_wheels(registered_right_front_wheel_handles_);
  halt_pods(registered_left_back_pod_handles_);
  halt_pods(registered_left_front_pod_handles_);
  halt_pods(registered_right_back_pod_handles_);
  halt_pods(registered_right_front_pod_handles_);
}

controller_interface::CallbackReturn LunaController::configure_wheel_segment(
  const std::string & side, const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names)
  {
    const auto interface_name = HW_IF_VELOCITY;
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&wheel_name, &interface_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LunaController::configure_pod_segment(
  const std::string & segment, const std::vector<std::string> & pod_names,
  std::vector<PodHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (pod_names.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' pod names specified", segment.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(pod_names.size());
  for (const auto & pod_name : pod_names)
  {
    const auto interface_name = HW_IF_POSITION;
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&pod_name, &interface_name](const auto & interface)
      {
        return interface.get_prefix_name() == pod_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", pod_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&pod_name](const auto & interface)
      {
        return interface.get_prefix_name() == pod_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", pod_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      PodHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace luna_control

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  luna_controller::LunaController, controller_interface::ControllerInterface)