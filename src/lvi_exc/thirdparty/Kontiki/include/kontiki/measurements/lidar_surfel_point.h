#ifndef LIDAR_SURFEL_POINT_H
#define LIDAR_SURFEL_POINT_H

#include <Eigen/Dense>

#include <iostream>
#include "../sensors/lidar.h"
#include "../trajectories/trajectory.h"
#include "../trajectory_estimator.h"
#include "ceres/rotation.h"

namespace kontiki {
namespace measurements {

template<typename LiDARModel>
class LiDARSurfelPoint {
 public:
  LiDARSurfelPoint(std::shared_ptr<LiDARModel> lidar, Eigen::Vector3d lidar_point, double* plane, double timestamp, double map_time,
                   double huber_loss, double weight)
      : lidar_(lidar), lidar_point_(lidar_point), plane_(plane), timestamp_(timestamp), map_time_(map_time),
        loss_function_(huber_loss), weight(weight) {}

  LiDARSurfelPoint(std::shared_ptr<LiDARModel> lidar, Eigen::Vector3d lidar_point, double* plane, double timestamp, double map_time, double huber_loss)
      : LiDARSurfelPoint(lidar, lidar_point, plane, timestamp, map_time, huber_loss, 1.0) {}

  LiDARSurfelPoint(std::shared_ptr<LiDARModel> lidar, Eigen::Vector3d lidar_point, double* plane, double timestamp, double map_time)
      : LiDARSurfelPoint(lidar, lidar_point, plane, timestamp, map_time, 5.) {}


  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 1, 1> reprojectPoint2map(const type::Trajectory<TrajectoryModel, T>& trajectory,
                                            const type::LiDAR<LiDARModel, T>& lidar,
                                            const T* plane_cp) const {
    int flags = trajectories::EvaluationFlags::EvalPosition | trajectories::EvaluationFlags::EvalOrientation;

    auto T_I0toG = trajectory.Evaluate(T(map_time_) + lidar.time_offset(), flags);
    auto T_IktoG = trajectory.Evaluate(T(timestamp_)+ lidar.time_offset(), flags);

    const Eigen::Matrix<T, 3, 1> p_LinI = lidar.relative_position();
    const Eigen::Quaternion<T> q_LtoI = lidar.relative_orientation();

    Eigen::Matrix<T, 3, 1> p_Lk = lidar_point_.cast<T>();//k时刻特征点在雷达系下的坐标
    Eigen::Matrix<T, 3, 1> p_I = q_LtoI * p_Lk + p_LinI;//k时刻特征点在IMU系下的坐标
    
    //旋转阵作用到向量上只是把向量进行了旋转，物理实体的变换还需加上原向量的坐标系原点在目标系下的位移
    //I0->P(向量)在I0下的坐标
    Eigen::Matrix<T, 3, 1> p_temp = T_I0toG->orientation.conjugate()*(
      T_IktoG->orientation * p_I + T_IktoG->position - T_I0toG->position);
    //P点在L0下（雷达地图坐标系）的坐标
    Eigen::Matrix<T, 3, 1> p_M = q_LtoI.conjugate() * (p_temp - p_LinI);
#if 1
    Eigen::Matrix<T,3,1> Pi = Eigen::Map<const Eigen::Matrix<T,3,1>>(plane_cp);
    T plane_d = T(Pi.norm());
    T plane_norm[3];
    plane_norm[0] = T(Pi[0])/plane_d;
    plane_norm[1] = T(Pi[1])/plane_d;
    plane_norm[2] = T(Pi[2])/plane_d;
    //向量内积求点到平面的距离
    T dist = ceres::DotProduct(plane_norm, p_M.data()) - plane_d;
#else
    T plane_d = T(plane_[3]);
    T plane_norm[3];
    plane_norm[0] = T(plane_[0]);
    plane_norm[1] = T(plane_[1]);
    plane_norm[2] = T(plane_[2]);
    T dist = ceres::DotProduct(plane_norm, p_M.data()) + plane_d;
#endif

    Eigen::Matrix<T, 1, 1> error(dist);
  //  error.topLeftCorner(1,1) = T(dist);
  //  Vector3 error_vector = dist * p_L / p_L.norm();

    return error;
  }

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 1, 1> Error(const type::Trajectory<TrajectoryModel, T> &trajectory,
                               const type::LiDAR<LiDARModel, T> &lidar,
                               const T* plane_cp) const {
    Eigen::Matrix<T, 1, 1> dist = reprojectPoint2map<TrajectoryModel, T>(trajectory, lidar, plane_cp);
    return T(weight) * (dist);
  }


  template<typename TrajectoryModel>
  Eigen::Matrix<double, 1, 1> point2plane(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    Eigen::Matrix<double, 1, 1> dist = reprojectPoint2map<TrajectoryModel, double>(trajectory, *lidar_, plane_);
    return dist;
  }

  void Lock(bool lock) {
    locked_ = lock;
  }

  bool IsLocked() {
    return locked_;
  }


  std::shared_ptr<LiDARModel> lidar_;

  Eigen::Vector3d lidar_point_;
  double* plane_; // double plane_[3]; //Eigen::Vector4d plane_;
  double timestamp_;
  double map_time_;
  double weight;

  bool locked_ = true;

 protected:

  template<typename TrajectoryModel>
  struct Residual {
    Residual(const LiDARSurfelPoint<LiDARModel> &m) : measurement(m){}

    /// 每次计算residual,都会根据优化后的参数重新构造traj和lidar sensor
    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      size_t offset = 0;
      auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);

      offset += trajectory_meta.NumParameters();
      auto lidar = entity::Map<LiDARModel, T>(&params[offset], lidar_meta);

      // measurements 里的plane与ceres优化量共享内存，所以不用取出来吧
      offset += lidar_meta.NumParameters();
      const T* plane = params[offset];

      Eigen::Map<Eigen::Matrix<T,1,1>> r(residual);
      r = measurement.Error<TrajectoryModel, T>(trajectory, lidar, plane);
      return true;
    }

    const LiDARSurfelPoint &measurement;
    typename TrajectoryModel::Meta trajectory_meta;
    typename LiDARModel::Meta lidar_meta;
  }; // Residual;
  
  //wz: 外部TrjectoryManager添加雷达观测的入口
  template<typename TrajectoryModel>
  void AddToEstimator(kontiki::TrajectoryEstimator<TrajectoryModel>& estimator) {

      using ResidualImpl = Residual<TrajectoryModel>;
      auto residual = new ResidualImpl(*this);
      auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
      std::vector<entity::ParameterInfo<double>> parameters;

      // Find timespans for the two observations
      double tmin, tmax;
      if(this->lidar_->TimeOffsetIsLocked()) {
          tmin = timestamp_;
          tmax = timestamp_;
      }
      else {
          tmin = timestamp_ - this->lidar_->max_time_offset();
          tmax = timestamp_ + this->lidar_->max_time_offset();
      }

      double map_min_time, map_max_time;
      if(this->lidar_->TimeOffsetIsLocked()) {
          map_min_time = map_time_;
          map_max_time = map_time_;
      }
      else {
          map_min_time = map_time_ - this->lidar_->max_time_offset();
          map_max_time = map_time_ + this->lidar_->max_time_offset();
      }

      /// A.1 先将轨迹参数添加到parameters中 (control points)
      estimator.AddTrajectoryForTimes({
                                        {map_min_time,map_max_time},
                                        {tmin, tmax}
                                      },
                                      residual->trajectory_meta,
                                      parameters);

      /// A.2 再将lidar传感器的参数添加到parameters中 (relative pose and timeoffset and so on ..(if have))
      lidar_->AddToProblem(estimator.problem(), {
                             {map_min_time,map_max_time},
                             {tmin, tmax}
                           },
                           residual->lidar_meta, parameters);

      /// A.3 最后将面片参数添加至 problem
      estimator.problem().AddParameterBlock(this->plane_, 3);

      parameters.push_back(entity::ParameterInfo<double>(this->plane_, 3));
      if (this->IsLocked()) {
        estimator.problem().SetParameterBlockConstant(this->plane_);
      }

      /// B.1 先往cost_function中添加待优化参数
      // Add parameters to cost function
      for (auto& pi : parameters) {
        cost_function->AddParameterBlock(pi.size);
      }
      // Add measurement info
      cost_function->SetNumResiduals(1);

      /// B.2 再添加residual
      // Give residual block to Problem
      std::vector<double*> params = entity::ParameterInfo<double>::ToParameterBlocks(parameters);

      estimator.problem().AddResidualBlock(cost_function,
                                           &loss_function_,
                                           params);
//      estimator.problem().AddResidualBlock(cost_function,
//                                           nullptr,
//                                           params);

//      estimator.problem().AddResidualBlock(cost_function,
//                                           nullptr,
//                                           entity::ParameterInfo<double>::ToParameterBlocks(parameters));

  }

  // The loss function is not a pointer since the Problem does not take ownership.
  ceres::HuberLoss loss_function_;

  template<template<typename> typename TrajectoryModel>
  friend class kontiki::TrajectoryEstimator;
};

} // namespace measurements
} // namespace kontiki

#endif // LIDAR_MEASUREMENT_H
