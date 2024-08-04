// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>

#include "base_traj_interpolator.h"
#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_LINEAR_JOINT_POSITION_TRAJ_INTERPOLATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_LINEAR_JOINT_POSITION_TRAJ_INTERPOLATOR_H_

namespace traj_utils {
class LinearJointPositionTrajInterpolator : public BaseTrajInterpolator {
 private:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector7i = Eigen::Matrix<int, 7, 1>;

  Vector7d q_start_;
  Vector7d q_goal_;

  Vector7d last_q_t_;
  Vector7d prev_q_goal_;

  double dt_;
  double last_time_;
  double max_time_;
  double start_time_;
  bool start_;
  bool first_goal_;

  double interpolation_fraction_;  // fraction of actual interpolation within an
                                   // interval

 public:
  inline LinearJointPositionTrajInterpolator()
      : dt_(0.),
        last_time_(0.),
        max_time_(1.),
        start_time_(0.),
        start_(false),
        first_goal_(true) {};

  inline ~LinearJointPositionTrajInterpolator() {};

  inline void Reset(const double &time_sec,
                    const Eigen::Matrix<double, 7, 1> &q_start,
                    const Eigen::Matrix<double, 7, 1> &q_goal,
                    const int &policy_rate, const int &rate,
                    const double &traj_interpolator_time_fraction) {
    dt_ = 1. / static_cast<double>(rate);
    last_time_ = time_sec;

    max_time_ =
        1. / static_cast<double>(policy_rate) * traj_interpolator_time_fraction;
    start_time_ = time_sec;

    start_ = false;

    if (first_goal_) {
      q_start_ = q_start;
      prev_q_goal_ = q_start;
      first_goal_ = false;
      // std::cout << "First goal of the interpolation" << std::endl;
    } else {
      prev_q_goal_ = q_goal_;
      q_start_ = prev_q_goal_;
    }
    // q_start_ = q_start; // why not this?
    q_goal_ = q_goal;

    std::cout << "RESET\n" << std::endl;
    std::cout << "q_start\n" << q_start.transpose() << std::endl;
    std::cout << "q_start_\n" << q_start_.transpose() << std::endl;
    std::cout << "q_goal\n" << q_goal.transpose() << std::endl;
    std::cout << "prev_q_goal_\n" << prev_q_goal_.transpose() << std::endl;
    std::cout << "max_time_\n" << max_time_ << std::endl;
    std::cout << "dt_\n" << dt_ << std::endl;
  };

  inline void GetNextStep(const double &time_sec, Vector7d &q_t) {
    if (!start_) {
      start_time_ = time_sec;
      last_q_t_ = q_start_;
      start_ = true;
    }
    // std::cout << q_start_.transpose() << " | " << q_goal_.transpose() <<
    // std::endl;

    if (last_time_ + dt_ <= time_sec) {
      double t =
          std::min(std::max((time_sec - start_time_) / max_time_, 0.), 1.);
      std::cout << "t\n" << q_start_.transpose() << std::endl;
      std::cout << "q_start_\n" << q_start_.transpose() << std::endl;
      std::cout << "q_goal_\n" << q_goal_.transpose() << std::endl;
      std::cout << "(q_goal_ - q_start_)\n"
                << (q_goal_ - q_start_).transpose() << std::endl;
      std::cout << "t * (q_goal_ - q_start_)\n"
                << (t * (q_goal_ - q_start_)).transpose() << std::endl;
      last_q_t_ = q_start_ + t * (q_goal_ - q_start_);
      last_time_ = time_sec;
    }
    q_t = last_q_t_;
    std::cout << "last_q_t_\n" << last_q_t_.transpose() << std::endl;
  };
};
}  // namespace traj_utils

#endif  // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_LINEAR_JOINT_POSITION_TRAJ_INTERPOLATOR_H_
