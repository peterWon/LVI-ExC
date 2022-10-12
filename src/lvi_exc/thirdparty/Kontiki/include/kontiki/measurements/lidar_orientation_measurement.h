//
// Created by hannes on 2017-11-29.
//

#ifndef KONTIKIV2_LIDAR_ORIENTATION_MEASUREMENT_H
#define KONTIKIV2_LIDAR_ORIENTATION_MEASUREMENT_H

#include <Eigen/Dense>

#include <iostream>
#include <kontiki/trajectories/trajectory.h>
#include <kontiki/trajectory_estimator.h>
#include "../sensors/lidar.h"

namespace kontiki {
namespace measurements {

template<typename LiDARModel>
class LiDAROrientationMeasurement {
  using Quat = Eigen::Quaternion<double>;
//  Eigen::Quaternion<T>
 public:
  LiDAROrientationMeasurement(
    std::shared_ptr<LiDARModel> lidar, double t, double lslam_t, double weight, const Quat &q)
    : lidar_(lidar), t_(t), lslam_start_t_(lslam_t), weight_(weight), q_(q), loss_function_(5.0) {}

  template<typename TrajectoryModel, typename T>
  Eigen::Quaternion<T> Measure(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                 const type::LiDAR<LiDARModel, T> &lidar) const {
    //t时刻载体（IMU）在世界系下的Pose
    auto Q_It_G = trajectory.Orientation(T(t_) + lidar.time_offset());
    auto Q_I0_G = trajectory.Orientation(T(lslam_start_t_) + lidar.time_offset());
    
    Eigen::Quaternion<T> q_It_I0 = Q_I0_G.conjugate() * Q_It_G;
    Eigen::Quaternion<T> q_LtoI = lidar.relative_orientation();
    Eigen::Quaternion<T> q_ItoL = q_LtoI.conjugate();
    Eigen::Quaternion<T> q_Lt_L0 = q_ItoL * q_It_I0 * q_LtoI;

    return q_Lt_L0;
  }

  template<typename TrajectoryModel, typename T>
  T Error(const type::Trajectory<TrajectoryModel, T> &trajectory, const type::LiDAR<LiDARModel, T> &lidar) const {
    Eigen::Quaternion<T> qhat = Measure<TrajectoryModel, T>(trajectory, lidar);
    return T(weight_)*(q_.cast<T>().angularDistance(qhat));
  }

  // Measurement data
  std::shared_ptr<LiDARModel> lidar_;
  double t_;
  double weight_;
  double lslam_start_t_;
  // double cs = 180./ M_PI;
  Quat q_;

 protected:

  // Residual struct for ceres-solver
  template<typename TrajectoryModel>
  struct Residual {
    Residual(const LiDAROrientationMeasurement<LiDARModel> &m) : measurement(m) {}

    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      size_t offset = 0;
      auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);

      offset += trajectory_meta.NumParameters();
      auto lidar = entity::Map<LiDARModel, T>(&params[offset], lidar_meta);

      residual[0] = measurement.Error<TrajectoryModel, T>(trajectory, lidar);
      return true;
    }

    const LiDAROrientationMeasurement& measurement;
    typename TrajectoryModel::Meta trajectory_meta;
    typename LiDARModel::Meta lidar_meta;
  }; // Residual;

  template<typename TrajectoryModel>
  void AddToEstimator(kontiki::TrajectoryEstimator<TrajectoryModel>& estimator) {
    using ResidualImpl = Residual<TrajectoryModel>;
    auto residual = new ResidualImpl(*this);
    auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
    std::vector<entity::ParameterInfo<double>> parameter_info;
    
    double tmin, tmax;
    if (this->lidar_->TimeOffsetIsLocked()) {
      tmin = t_;
      tmax = t_;
    }else {
      tmin = t_ - this->lidar_->max_time_offset();
      tmax = t_ + this->lidar_->max_time_offset();
    }
    double map_min_time, map_max_time;
    if(this->lidar_->TimeOffsetIsLocked()) {
      map_min_time = lslam_start_t_;
      map_max_time = lslam_start_t_;
    }else {
      map_min_time = lslam_start_t_ - this->lidar_->max_time_offset();
      map_max_time = lslam_start_t_ + this->lidar_->max_time_offset();
    }
    // Add trajectory to problem
    //estimator.trajectory()->AddToProblem(estimator.problem(), residual->meta, parameter_blocks, parameter_sizes);
    estimator.AddTrajectoryForTimes(
      {{map_min_time,map_max_time},{tmin, tmax}}, 
      residual->trajectory_meta, parameter_info);
    lidar_->AddToProblem(
      estimator.problem(), 
      {{map_min_time,map_max_time},{tmin, tmax}}, 
      residual->lidar_meta, parameter_info);

    for (auto& pi : parameter_info) {
      cost_function->AddParameterBlock(pi.size);
    }

    // Add measurement
    cost_function->SetNumResiduals(1);
    // If we had any measurement parameters to set, this would be the place

    // Give residual block to estimator problem
    estimator.problem().AddResidualBlock(
      cost_function, &loss_function_,
      entity::ParameterInfo<double>::ToParameterBlocks(parameter_info));
  }
  ceres::HuberLoss loss_function_;
  // TrajectoryEstimator must be a friend to access protected members
  template<template<typename> typename TrajectoryModel>
  friend class kontiki::TrajectoryEstimator;
};

} // namespace measurements
} // namespace kontiki


#endif //KONTIKIV2_LIDAR_ORIENTATION_MEASUREMENT_H
