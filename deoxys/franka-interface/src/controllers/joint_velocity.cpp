// Copyright 2022 Yifeng Zhu

#include "controllers/joint_velocity.h"

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "franka_controller.pb.h"
#include "franka_robot_state.pb.h"
#include "utils/common_utils.h"
#include "utils/control_utils.h"
#include "utils/robot_utils.h"

namespace controller {
JointVelocityController::JointVelocityController() {}
JointVelocityController::~JointVelocityController() {}

JointVelocityController::JointVelocityController(franka::Model &model) {
  model_ = &model;
}

bool JointVelocityController::ParseMessage(const FrankaControlMessage &msg) {
  if (!msg.control_msg().UnpackTo(&control_msg_)) {
    return false;
  }

  // Kp << control_msg_.kp();
  // Kd << control_msg_.kd();

  std::vector<double> kp_array;
  std::vector<double> kd_array;

  kp_array.reserve(control_msg_.kp().size());
  kd_array.reserve(control_msg_.kd().size());

  for (double kp_i : control_msg_.kp()) {
    kp_array.push_back(kp_i);
  }
  for (double kd_i : control_msg_.kd()) {
    kd_array.push_back(kd_i);
  }

  Kp << Eigen::Map<const Eigen::Matrix<double, 7, 1>>(kp_array.data());
  Kd << Eigen::Map<const Eigen::Matrix<double, 7, 1>>(kd_array.data());

  joint_max_ << 2.8978, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  joint_min_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

  velocity_max_ << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  velocity_min_ << -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0;

  this->state_estimator_ptr_->ParseMessage(msg.state_estimator_msg());
  return true;
}

void JointVelocityController::ComputeGoal(
    const std::shared_ptr<StateInfo> &current_state_info,
    std::shared_ptr<StateInfo> &goal_state_info) {
  if (control_msg_.goal().is_delta()) {
    Eigen::Matrix<double, 7, 1> delta_joint_position;
    delta_joint_position << control_msg_.goal().q1(), control_msg_.goal().q2(),
        control_msg_.goal().q3(), control_msg_.goal().q4(),
        control_msg_.goal().q5(), control_msg_.goal().q6(),
        control_msg_.goal().q7();
    goal_state_info->joint_positions =
        current_state_info->joint_positions + delta_joint_position;
  } else {
    goal_state_info->joint_positions << control_msg_.goal().q1(),
        control_msg_.goal().q2(), control_msg_.goal().q3(),
        control_msg_.goal().q4(), control_msg_.goal().q5(),
        control_msg_.goal().q6(), control_msg_.goal().q7();
  }
  // goal_state_info->joint_positions << control_msg_.goal().q1(),
  // control_msg_.goal().q2(), control_msg_.goal().q3(),
  // control_msg_.goal().q4(), control_msg_.goal().q5(),
  // control_msg_.goal().q6(), control_msg_.goal().q7();
}

std::array<double, 7> JointVelocityController::Step(
    const franka::RobotState &robot_state,
    const Eigen::Matrix<double, 7, 1> &desired_q) {
  std::array<double, 7> qv_d_array{};
  Eigen::VectorXd::Map(&qv_d_array[0], 7) = desired_q;
  return qv_d_array;
}
}  // namespace controller
