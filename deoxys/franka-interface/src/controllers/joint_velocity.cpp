// Copyright 2022 Yifeng Zhu

#include "franka_controller.pb.h"
#include "franka_robot_state.pb.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "utils/common_utils.h"
#include "utils/control_utils.h"
#include "utils/robot_utils.h"

#include "controllers/joint_velocity.h"

#include <memory>

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

std::array<double, 7>
JointVelocityController::Step(const franka::RobotState &robot_state,
                               const Eigen::Matrix<double, 7, 1> &desired_q) {

  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();

  Eigen::Matrix<double, 7, 1> current_q, current_dq;

  current_q = this->state_estimator_ptr_->GetCurrentJointPos();
  current_dq = this->state_estimator_ptr_->GetCurrentJointVel();

  Eigen::MatrixXd joint_pos_error(7, 1);
  joint_pos_error << desired_q - current_q;

  Eigen::Matrix<double, 7, 1> desired_qv = desired_q - current_q;

  double kv = 2.0
  desired_qv *=kv

  std:: cout << "desired_qv\n" << desired_qv.transpose() << std::endl;
  
  for (int i = 0; i < 7; i++) {
    if (desired_qv[i] < velocity_min_[i]){
      desired_qv[i] = velocity_min_[i];
    }

    if (desired_qv[i] > velocity_max_[i]){
      desired_qv[i] = velocity_max_[i];
    }
  }
  std:: cout << "v max desired_qv\n" << desired_qv.transpose() << std::endl;


  Eigen::Matrix<double, 7, 1> dist2joint_max;
  Eigen::Matrix<double, 7, 1> dist2joint_min;

  dist2joint_max = joint_max_.matrix() - current_q;
  dist2joint_min = current_q - joint_min_.matrix();

  for (int i = 0; i < 7; i++) {
    if (dist2joint_max[i] < 0.1 && desired_qv[i] > 0.0)
      desired_qv[i] = 0.0;
    if (dist2joint_min[i] < 0.1 && desired_qv[i] < 0.0)
      desired_qv[i] = 0.0;
  }

  std:: cout << "d max desired_qv\n" << desired_qv.transpose() << std::endl;


  std::array<double, 7> qv_d_array{};
  Eigen::VectorXd::Map(&qv_d_array[0], 7) = desired_qv;
  return qv_d_array;

}
} // namespace controller
