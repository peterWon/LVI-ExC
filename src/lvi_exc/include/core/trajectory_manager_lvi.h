/*
 * LI_Calib: An Open Platform for LiDAR-IMU Calibration
 * Copyright (C) 2020 Jiajun Lv
 * Copyright (C) 2020 Kewei Hu
 * Copyright (C) 2020 Jinhong Xu
 * Copyright (C) 2020 LI_Calib Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H
#include <kontiki/measurements/static_rscamera_measurement.h>
#include <kontiki/measurements/lifting_rscamera_measurement.h>
#include <kontiki/measurements/newton_rscamera_measurement.h>
#include <kontiki/sensors/constant_bias_imu.h>
#include <kontiki/sensors/vlp16_lidar.h>
#include <kontiki/sensors/pinhole_camera.h>

#include <kontiki/trajectory_estimator.h>
#include <kontiki/trajectories/split_trajectory.h>
#include <kontiki/trajectories/uniform_r3_spline_trajectory.h>
#include <kontiki/trajectories/uniform_so3_spline_trajectory.h>

#include <kontiki/measurements/gyroscope_measurement.h>
#include <kontiki/measurements/accelerometer_measurement.h>
#include <kontiki/measurements/lidar_surfel_point.h>

#include <kontiki/measurements/orientation_measurement.h>
#include <kontiki/measurements/position_measurement.h>
#include <kontiki/measurements/lidar_position_measurement.h>
#include <kontiki/measurements/lidar_orientation_measurement.h>
#include <kontiki/measurements/camera_surfel_landmark.h>

#include <kontiki/sfm/sfm.h>

#include <utils/dataset_reader.h>
#include <core/surfel_association.h>
#include <core/calibration.hpp>
#include <Eigen/Core>
#include <fstream>
#include <memory>

#include <glog/logging.h>

namespace licalib {
struct CameraIntrinsic{
  int row = 720;
  int col = 1280;
  double readout = 0.0666;
  double k1 = -0.01832333;
  double k2 = -0.00041966;
  double p1 = 0.00006292;
  double p2 = 0.00141579;
  double k3 = 0.00141579;
  double fx = 365.56114295;
  double fy = 367.2624234;
  double cx = 623.56879222;
  double cy = 379.80700248;
};

class TrajectoryManagerLVI {
  using IMUSensor = kontiki::sensors::ConstantBiasImu;
  using LiDARSensor = kontiki::sensors::VLP16LiDAR;
  using CameraSensor = kontiki::sensors::PinholeCamera;
  
  using SplitTrajectory = kontiki::trajectories::SplitTrajectory;
  using SO3Trajectory = kontiki::trajectories::UniformSO3SplineTrajectory;
  using R3Trajectory = kontiki::trajectories::UniformR3SplineTrajectory;

  using SO3TrajEstimator   = kontiki::TrajectoryEstimator<
    kontiki::trajectories::UniformSO3SplineTrajectory>;
  using R3TrajEstimator    = kontiki::TrajectoryEstimator<
    kontiki::trajectories::UniformR3SplineTrajectory>;
  using SplitTrajEstimator = kontiki::TrajectoryEstimator<
    kontiki::trajectories::SplitTrajectory>;
  
  using GyroMeasurement = kontiki::measurements::GyroscopeMeasurement<IMUSensor>;
  using AccelMeasurement = kontiki::measurements::AccelerometerMeasurement<IMUSensor>;
  using SurfMeasurement = kontiki::measurements::LiDARSurfelPoint<LiDARSensor>;
  using CameraMeasurement = kontiki::measurements::StaticRsCameraMeasurement<CameraSensor>;
  using CameraSurfMeasurement = kontiki::measurements::CameraSurfelLandmark<CameraSensor,LiDARSensor>;

  using OrientationMeasurement  = kontiki::measurements::OrientationMeasurement;
  using PositionMeasurement     = kontiki::measurements::PositionMeasurement;
  using LidarOrientationMeasurement  = kontiki::measurements::LiDAROrientationMeasurement<LiDARSensor>;
  using LidarPositionMeasurement     = kontiki::measurements::LiDARPositionMeasurement<LiDARSensor>;

public:
  typedef std::shared_ptr<TrajectoryManagerLVI> Ptr;
  using Result = std::unique_ptr<kontiki::trajectories::TrajectoryEvaluation<double>>;

  explicit TrajectoryManagerLVI(const CameraIntrinsic& ci,
      double start_time, double end_time, double knot_distance,
      double time_offset_padding)
          : time_offset_padding_(time_offset_padding),
            map_time_(0),
            imu_(std::make_shared<IMUSensor>()),
            lidar_(std::make_shared<LiDARSensor>()),
            calib_param_manager(std::make_shared<CalibParamManager>()) {
    assert(knot_distance > 0 && "knot_distance should be lager than 0");
    
    camera_ = std::make_shared<CameraSensor>(
      CameraSensor(ci.row, ci.col, ci.readout, 
                   ci.k1, ci.k2, ci.p1, ci.p2,
                   ci.k3, 
                   ci.fx, ci.fy, ci.cx, ci.cy));
    camera_->set_max_time_offset(0.001);
    lidar_->set_max_time_offset(0.001); //default 0.01
    double traj_start_time = start_time - time_offset_padding;
    double traj_end_time = end_time + time_offset_padding;
    traj_ = std::make_shared<kontiki::trajectories::SplitTrajectory>
            (knot_distance, knot_distance, traj_start_time, traj_start_time);
    initialTrajTo(traj_end_time);
  }
 
  void initialTrajTo(double max_time);
  
  //call after SO(3) initialization
  void setGravityGuess(const Eigen::Vector4d& g_t){
    Eigen::Vector3d g = g_t.segment<3>(0);
    Eigen::Vector3d WG;
    WG << 0, 0, -9.79;
    // int flags = trajectories::EvaluationFlags::EvalOrientation;
    //t时刻IMU的Pose
    auto Q_t = traj_->Orientation(g_t[3]);
    Eigen::Vector3d g_0 = Q_t.toRotationMatrix() * g;
    
    Eigen::Quaterniond Q = Eigen::Quaterniond::FromTwoVectors(g_0, WG);
    Eigen::Vector3d rpy = Q.toRotationMatrix().eulerAngles(0,1,2);
    LOG(INFO)<<g.transpose()<<","<<g_0.transpose();
    LOG(INFO)<<rpy[0]<<","<<rpy[1]<<","<<rpy[2];
    imu_->set_gravity_orientation_roll(rpy[2]);
    imu_->set_gravity_orientation_pitch(rpy[1]);
    //TODO
    //这样设置，IMU模型算出来的重力跟输入的重力后两位差了一个负号，尚不清楚两者的物理含义是否一致
    // calib_param_manager->set_gravity(g_0);
    calib_param_manager->set_gravity(imu_->refined_gravity());
  }
  
  void feedIMUData(const IO::IMUData& data);


  void initialSO3TrajWithGyro();
  void initialTrajWithIMUReadings(); 

  void trajInitFromSurfel(SurfelAssociation::Ptr surfels_association,
                          bool opt_time_offset_ = false);

  void trajInitFromLidarPose(const std::vector<
    std::pair<double, Eigen::Matrix4d>>& lidar_poses,
    double lidar_start_time,
    bool opt_time_offset_= false);

  void trajInitFromVisualFrames(const std::map<
    int64_t, std::shared_ptr<kontiki::sfm::View>>& frames,
    bool opt_time_offset_ = false);

  void trajInitFromLVIdata(const std::map<
    int64_t, std::shared_ptr<kontiki::sfm::View>>& frames,
    const std::vector<std::pair<double, Eigen::Matrix4d>>& lidar_poses,
    double lidar_start_time,
    bool opt_time_offset_ = false);
    
  void trajInitFromLVIdata(
    const std::map<int64_t, std::shared_ptr<kontiki::sfm::View>>& frames,
    SurfelAssociation::Ptr surfels_association,
    bool opt_time_offset_ = false,
    bool lock_traj_and_lidar = false);
  
  void trajInitFromLVIdata(
    const std::map<int64_t, std::shared_ptr<kontiki::sfm::View>>& frames,
    SurfelAssociation::Ptr surfels_association,
    const std::map<kontiki::sfm::Landmark*, size_t>& lm_surfel,
    bool opt_time_offset_ = false,
    bool lock_traj_and_lidar = false);

  bool evaluateIMUPose(double imu_time, int flags, Result& result) const;

  bool evaluateLidarPose(double lidar_time, Eigen::Quaterniond& q_LtoG,
                         Eigen::Vector3d& p_LinG) const;

  bool evaluateCameraPose(double camera_time, Eigen::Quaterniond& q_CtoG,
                         Eigen::Vector3d& p_CinG) const;

  bool evaluateLidarRelativeRotation(double lidar_time1, double lidar_time2,
                                     Eigen::Quaterniond& q_L2toL1) const;
  
  bool evaluateCameraRelativeRotation(double camera_time1, double camera_time2,
                                     Eigen::Quaterniond& q_C2toC1) const;

  CalibParamManager::Ptr getCalibParamManager() const {
    return calib_param_manager;
  }

  double get_map_time() const {
    return map_time_;
  }

  /// Access the trajectory
  std::shared_ptr<kontiki::trajectories::SplitTrajectory> getTrajectory() const {
    return traj_;
  }

  std::shared_ptr<kontiki::sensors::PinholeCamera> getCameraModel() const{
    return camera_;
  }

  //第一维为个数  
  Eigen::Vector2d err_surfel_;

  Eigen::Vector4d err_acc_;
  Eigen::Vector4d err_gyr_;
  
  Eigen::Vector3d err_cam_;

private:
  template <typename TrajectoryModel>
  void addGyroscopeMeasurements(
          std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator);

  template <typename TrajectoryModel>
  void addAccelerometerMeasurement(
          std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator);

  template <typename TrajectoryModel>
  void addSurfMeasurement(
          std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
          const SurfelAssociation::Ptr surfel_association);

  template <typename TrajectoryModel>
  void addVisualObservation(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
        const std::map<int64_t, std::shared_ptr<kontiki::sfm::View>>& frames);

  template <typename TrajectoryModel>
  void addVisualLidarMeasurement(
    std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
    const std::map<kontiki::sfm::Landmark*, size_t>& lm_surfel); 
  
  template <typename TrajectoryModel>
  void addLidarPoses(
      std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
      double lidar_start_time,
      const std::vector<std::pair<double, Eigen::Matrix4d>>& lidar_poses);

  template <typename TrajectoryModel>
  void addCallback(
          std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator);

  void printErrorStatistics(const std::string& intro, bool show_gyro = true,
                            bool show_accel = true, bool show_lidar = true,
                            bool show_lidar_pose = true,
                            bool show_camera = true);

  double map_time_;
  double time_offset_padding_;
  std::shared_ptr<kontiki::trajectories::SplitTrajectory> traj_;
  std::shared_ptr<kontiki::sensors::ConstantBiasImu> imu_;
  std::shared_ptr<kontiki::sensors::VLP16LiDAR> lidar_;
  std::shared_ptr<kontiki::sensors::PinholeCamera> camera_;

  CalibParamManager::Ptr calib_param_manager;

  std::vector<IO::IMUData> imu_data_;

  Eigen::aligned_vector<Eigen::Vector3d> closest_point_vec_;

  std::vector< std::shared_ptr<GyroMeasurement>>  gyro_list_;
  std::vector< std::shared_ptr<AccelMeasurement>> accel_list_;
  std::vector< std::shared_ptr<LidarOrientationMeasurement>> lori_list_;
  std::vector< std::shared_ptr<LidarPositionMeasurement>> lpos_list_;
  std::vector< std::shared_ptr<SurfMeasurement>>  surfelpoint_list_;
  std::vector< std::shared_ptr<CameraMeasurement>>  landmark_list_;
  std::vector< std::shared_ptr<CameraSurfMeasurement>>  surf_landmark_list_;
};
}

#endif
