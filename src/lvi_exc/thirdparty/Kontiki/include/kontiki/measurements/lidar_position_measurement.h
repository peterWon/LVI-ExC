//
// Created by hannes on 2017-11-29.
//

#ifndef KONTIKIV2_LIDAR_POSITION_MEASUREMENT_H
#define KONTIKIV2_LIDAR_POSITION_MEASUREMENT_H

#include <Eigen/Dense>

#include <iostream>
#include <kontiki/trajectories/trajectory.h>
#include <kontiki/trajectory_estimator.h>
#include "../sensors/lidar.h"
#include "ceres/rotation.h"

namespace kontiki {
namespace measurements {

template<typename LiDARModel>
class LiDARPositionMeasurement {
  using Vector3 = Eigen::Matrix<double, 3, 1>;
 public:
  LiDARPositionMeasurement(
    std::shared_ptr<LiDARModel> lidar, double t, double lidar_start_time,
    double weight, const Vector3 &p, Eigen::Quaterniond q)
    : lidar_(lidar), t_(t), lslam_start_t_(lidar_start_time), weight_(weight), p_(p), q_(q), loss_function_(5.0) {}

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Measure(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                 const type::LiDAR<LiDARModel, T> &lidar) const {
    int flags = trajectories::EvaluationFlags::EvalPosition | 
                trajectories::EvaluationFlags::EvalOrientation;
    //k时刻载体（IMU）在世界系下的Pose
    auto T_Ik_G = trajectory.Evaluate(T(t_) + lidar.time_offset() , flags);
    auto T_I0_G = trajectory.Evaluate(T(lslam_start_t_) + lidar.time_offset(), flags);

    Eigen::Quaternion<T> q_Ik_I0_est = T_I0_G->orientation.conjugate()
      * T_Ik_G->orientation;
    Eigen::Matrix<T, 3, 1> p_Ik_I0_est = T_I0_G->orientation.conjugate()
      * (T_Ik_G->position - T_I0_G->position);

    //Lidar在IMU下的Pose
    const Eigen::Matrix<T, 3, 1> p_LinI = lidar.relative_position();
    const Eigen::Quaternion<T> q_LtoI = lidar.relative_orientation();
    
    //IMU在Lidar下的位置
    Eigen::Matrix<T, 3, 1> p_IinL = q_LtoI.conjugate() * (-p_LinI);
    
    //t0: LiDAR-SLAM起始时刻, tk位姿观测时刻
    Eigen::Matrix<T, 3, 1> p_Lk_L0 = p_.cast<T>();
    Eigen::Quaternion<T> q_Lk_L0 = q_.cast<T>();
    
    Eigen::Matrix<T, 3, 1> v_I0Ik_I0 = T_I0_G->orientation.conjugate()
      * (T_Ik_G->position - T_I0_G->position);
    Eigen::Matrix<T, 3, 1> v_L0I0_I0 = q_LtoI * p_IinL;
    Eigen::Matrix<T, 3, 1> v_LkIk_I0 = (T_I0_G->orientation.conjugate()
      * T_Ik_G->orientation) * p_LinI;
    
    Eigen::Matrix<T, 3, 1> v_L0Lk_L0 = 
      q_LtoI.conjugate() * (v_I0Ik_I0 + v_L0I0_I0 + v_LkIk_I0);

  
    Eigen::Matrix<T, 3, 1> error = v_L0Lk_L0 - p_Lk_L0;

    return error;
  }

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Error(const type::Trajectory<TrajectoryModel, T> &trajectory,
                               const type::LiDAR<LiDARModel, T> &lidar) const {
    Eigen::Matrix<T, 3, 1> p_M_L = p_.cast<T>();
    // return T(weight_) * (p_M_L - Measure<TrajectoryModel, T>(trajectory, lidar));
    return T(weight_) * (Measure<TrajectoryModel, T>(trajectory, lidar));
  }


  template<typename TrajectoryModel>
  Eigen::Matrix<double, 3, 1> Error(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    return weight_*(Measure<TrajectoryModel, double>(trajectory, *lidar_));
  }

  // Measurement data
  std::shared_ptr<LiDARModel> lidar_;
  double t_;
  double lslam_start_t_;
  Vector3 p_;
  Eigen::Quaterniond q_;
  double weight_;

 protected:

  // Residual struct for ceres-solver
  template<typename TrajectoryModel>
  struct Residual {
    Residual(const LiDARPositionMeasurement<LiDARModel> &m) : measurement(m) {}

    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      size_t offset = 0;
      auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);

      offset += trajectory_meta.NumParameters();
      auto lidar = entity::Map<LiDARModel, T>(&params[offset], lidar_meta);

      Eigen::Map<Eigen::Matrix<T,3,1>> r(residual);
      r = measurement.Error<TrajectoryModel, T>(trajectory, lidar);
     
      return true;
    }

    const LiDARPositionMeasurement& measurement;
    typename TrajectoryModel::Meta trajectory_meta;
    typename LiDARModel::Meta lidar_meta;
  }; // Residual;

  template<typename TrajectoryModel>
  void AddToEstimator(kontiki::TrajectoryEstimator<TrajectoryModel>& estimator) {
    using ResidualImpl = Residual<TrajectoryModel>;
    auto residual = new ResidualImpl(*this);
    auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
    std::vector<entity::ParameterInfo<double>> parameter_info;

    // Add trajectory to problem
    //estimator.trajectory()->AddToProblem(estimator.problem(), residual->meta, parameter_blocks, parameter_sizes);
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

    estimator.AddTrajectoryForTimes({{map_min_time,map_max_time},{tmin,tmax}}, 
      residual->trajectory_meta, parameter_info);

    lidar_->AddToProblem(estimator.problem(), { 
      {map_min_time,map_max_time},{tmin,tmax}}, 
      residual->lidar_meta, parameter_info);
    
    for (auto& pi : parameter_info) {
      cost_function->AddParameterBlock(pi.size);
    }

    // Add measurement
    cost_function->SetNumResiduals(3);
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


#endif //KONTIKIV2_LIDAR_POSITION_MEASUREMENT_H
