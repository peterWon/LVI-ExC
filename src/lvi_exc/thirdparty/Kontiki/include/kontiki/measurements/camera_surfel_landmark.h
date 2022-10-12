#ifndef CAMERA_SURFEL_LANDMARK_H
#define CAMERA_SURFEL_LANDMARK_H

#include <Eigen/Dense>

#include <iostream>
#include "../sensors/camera.h"
#include "../sfm/sfm.h"
#include "../trajectories/trajectory.h"
#include "../trajectory_estimator.h"
#include "ceres/rotation.h"

namespace kontiki {
namespace measurements {

template<typename CameraModel, typename LiDARModel>
class CameraSurfelLandmark {
 public:
  CameraSurfelLandmark(
      std::shared_ptr<CameraModel> camera, 
      std::shared_ptr<LiDARModel> lidar, 
      kontiki::sfm::Landmark* landmark, double* plane, 
      double timestamp, double map_time,
      double huber_loss, double weight) : camera_(camera), lidar_(lidar), 
      landmark_(landmark), plane_(plane), timestamp_(timestamp), 
      map_time_(map_time), loss_function_(huber_loss), weight(weight) {}

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 1, 1> reprojectPoint2map(
      const type::Trajectory<TrajectoryModel, T>& trajectory,
      const type::Camera<CameraModel, T>& camera,
      const type::LiDAR<LiDARModel, T>& lidar,
      const T* plane_cp,
      const T inverse_depth) const {
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Vector2 = Eigen::Matrix<T, 2, 1>;
    const T eps = T(1e-8);
    int flags = trajectories::EvaluationFlags::EvalPosition | 
                trajectories::EvaluationFlags::EvalOrientation;

    auto T_I0toG = trajectory.Evaluate(T(map_time_) + camera.time_offset(), flags);
    auto T_IktoG = trajectory.Evaluate(T(timestamp_)+ camera.time_offset(), flags);

    const Eigen::Matrix<T, 3, 1> p_CinI = camera.relative_position();
    const Eigen::Matrix<T, 3, 1> p_LinI = lidar.relative_position();
    const Eigen::Quaternion<T> q_CtoI = camera.relative_orientation();
    const Eigen::Quaternion<T> q_LtoI = lidar.relative_orientation();
    //IMU在Camera下的姿态
    const Vector3 p_ct = q_CtoI.conjugate()*(-p_CinI);
    const Eigen::Quaternion<T> q_ct = q_CtoI.conjugate();
    
    //目标点在IMU下的坐标
    Vector2 y = landmark_->reference()->uv().cast<T>();
    Vector3 yh = camera.Unproject(y) / (inverse_depth + eps);
    //空间点归一化坐标转到IMU系
    // Vector3 p_I = q_ct.conjugate() * (yh - inverse_depth * p_ct);
    Eigen::Matrix<T, 3, 1> p_I = q_CtoI * yh + p_CinI;//k时刻landmark在IMU系下的坐标

    // Eigen::Matrix<T, 3, 1> p_Ck = camera_point_.cast<T>();//k时刻landmark在相机系下的坐标
    // Eigen::Matrix<T, 3, 1> p_I = q_CtoI * p_Ck + p_CinI;//k时刻landmark在IMU系下的坐标
    
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
  Eigen::Matrix<T, 1, 1> Error(
      const type::Trajectory<TrajectoryModel, T> &trajectory,
      const type::Camera<CameraModel, T> &camera,
      const type::LiDAR<LiDARModel, T> &lidar,
      const T* plane_cp,
      const T inverse_depth) const {
    Eigen::Matrix<T, 1, 1> dist = reprojectPoint2map<TrajectoryModel, T>(
      trajectory, camera, lidar, plane_cp, inverse_depth);
    return T(weight) * (dist);
  }


  template<typename TrajectoryModel>
  Eigen::Matrix<double, 1, 1> point2plane(
      const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    Eigen::Matrix<double, 1, 1> dist = reprojectPoint2map<
      TrajectoryModel, double>(trajectory, *camera_, plane_);
    return dist;
  }

  void Lock(bool lock) {
    locked_ = lock;
  }

  bool IsLocked() {
    return locked_;
  }


  std::shared_ptr<CameraModel> camera_;
  std::shared_ptr<LiDARModel> lidar_;

  // Eigen::Vector3d camera_point_;
  kontiki::sfm::Landmark* landmark_;
  double* plane_; // double plane_[3]; //Eigen::Vector4d plane_;
  double timestamp_; //landmark的reference frame的时间戳
  double map_time_;
  double weight;

  bool locked_ = true;

 protected:

  template<typename TrajectoryModel>
  struct Residual {
    Residual(const CameraSurfelLandmark<CameraModel, LiDARModel> &m) 
      : measurement(m){}

    /// 每次计算residual,都会根据优化后的参数重新构造traj和camera sensor
    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      size_t offset = 0;
      auto trajectory = entity::Map<TrajectoryModel, T>(
        &params[offset], trajectory_meta);

      offset += trajectory_meta.NumParameters();
      auto camera = entity::Map<CameraModel, T>(&params[offset], camera_meta);

      offset += camera_meta.NumParameters();
      auto lidar = entity::Map<LiDARModel, T>(&params[offset], lidar_meta);

      // measurements 里的plane与ceres优化量共享内存，所以不用取出来吧
      offset += lidar_meta.NumParameters();
      const T* plane = params[offset];
      
      // offset += 3;
      // T inverse_depth = params[offset][0];
      T inverse_depth = T(measurement.landmark_->inverse_depth());

      Eigen::Map<Eigen::Matrix<T,1,1>> r(residual);
      r = measurement.Error<TrajectoryModel, T>(
        trajectory, camera, lidar, plane, inverse_depth);
      return true;
    }

    const CameraSurfelLandmark &measurement;
    typename TrajectoryModel::Meta trajectory_meta;
    typename CameraModel::Meta camera_meta;
    typename LiDARModel::Meta lidar_meta;
  }; // Residual;
  
  //wz: 外部TrjectoryManager添加雷达观测的入口
  template<typename TrajectoryModel>
  void AddToEstimator(kontiki::TrajectoryEstimator<TrajectoryModel>& estimator){

      using ResidualImpl = Residual<TrajectoryModel>;
      auto residual = new ResidualImpl(*this);
      auto cost_function = new ceres::DynamicAutoDiffCostFunction<
        ResidualImpl>(residual);

      std::vector<entity::ParameterInfo<double>> parameters;
      // std::cout<<"finish adding to problem 0!\n";
      // Find timespans for the two observations
      double tmin, tmax;
      if(this->camera_->TimeOffsetIsLocked()) {
        tmin = timestamp_;
        tmax = timestamp_;
      }else {
        tmin = timestamp_ - this->camera_->max_time_offset();
        tmax = timestamp_ + this->camera_->max_time_offset();
      }

      double map_min_time, map_max_time;
      if(this->camera_->TimeOffsetIsLocked()) {
        map_min_time = map_time_;
        map_max_time = map_time_;
      }else {
        map_min_time = map_time_ - this->camera_->max_time_offset();
        map_max_time = map_time_ + this->camera_->max_time_offset();
      }
      // std::cout<<"finish adding to problem 1!\n";
      /// A.1 先将轨迹参数添加到parameters中 (control points)
      estimator.AddTrajectoryForTimes({{map_min_time,map_max_time},{tmin, tmax}},
                                      residual->trajectory_meta,
                                      parameters);
      // std::cout<<"finish adding to problem 2!\n";
      /// A.2 再将传感器的参数添加到parameters中 
      // relative pose and timeoffset and so on ..(if have)
      camera_->AddToProblem(estimator.problem(), 
                            {{map_min_time,map_max_time},
                             {tmin, tmax}},
                           residual->camera_meta, parameters);
      // std::cout<<"finish adding to problem 3!\n";
      lidar_->AddToProblem(estimator.problem(), 
                            {{map_min_time,map_max_time},
                             {tmin, tmax}},
                           residual->lidar_meta, parameters);
      // std::cout<<"finish adding to problem 4!\n";
      /// A.3 将面片参数添加至 problem
      estimator.problem().AddParameterBlock(this->plane_, 3);
      parameters.push_back(entity::ParameterInfo<double>(this->plane_, 3));
      if (this->IsLocked()) {
        estimator.problem().SetParameterBlockConstant(this->plane_);
      }
      // std::cout<<"finish adding to problem 5!\n";
      // A.4 Add Landmark inverse depth
      double *p_rho = landmark_->inverse_depth_ptr();
      estimator.problem().AddParameterBlock(p_rho, 1);
      estimator.problem().SetParameterLowerBound(p_rho, 0, 0.);
      parameters.push_back(entity::ParameterInfo<double>(p_rho, 1));
      if (landmark_->IsLocked()) {
        estimator.problem().SetParameterBlockConstant(p_rho);
      }
      // std::cout<<"finish adding to problem 6!\n";
      /// B.1 先往cost_function中添加待优化参数
      // Add parameters to cost function
      for (auto& pi : parameters) {
        cost_function->AddParameterBlock(pi.size);
      }
      // Add measurement info
      cost_function->SetNumResiduals(1);
      // std::cout<<"finish adding to problem 7!\n";
      /// B.2 再添加residual
      // Give residual block to Problem
      std::vector<double*> params = 
        entity::ParameterInfo<double>::ToParameterBlocks(parameters);

      estimator.problem().AddResidualBlock(cost_function,
                                           &loss_function_,
                                           params);
      // std::cout<<"finish adding to problem 8!\n";
  }

  // The loss function is not a pointer since the Problem does not take ownership.
  ceres::HuberLoss loss_function_;

  template<template<typename> typename TrajectoryModel>
  friend class kontiki::TrajectoryEstimator;
};

} // namespace measurements
} // namespace kontiki

#endif // CAMERA_SURFEL_LANDMARK_H
