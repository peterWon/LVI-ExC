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
#include <core/trajectory_manager_lvi.h>
#include <utils/math_utils.h>
#include <utils/eigen_utils.hpp>
#include <utils/ceres_callbacks.h>

#include <memory>

namespace licalib {
using namespace kontiki::trajectories;

void TrajectoryManagerLVI::initialTrajTo(double max_time) {
  Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p0(0,0,0);
  traj_->R3Spline()->ExtendTo(max_time, p0);
  traj_->SO3Spline()->ExtendTo(max_time, q0);
}

void TrajectoryManagerLVI::feedIMUData(const IO::IMUData& data) {
  imu_data_.emplace_back(data);
}


void TrajectoryManagerLVI::initialSO3TrajWithGyro() {
  assert(imu_data_.size() > 0 &&
         "[initialSO3TrajWithGyro]: There's NO imu data for initialization.");
  std::shared_ptr<SO3TrajEstimator> estimator_SO3;
  estimator_SO3 = std::make_shared<SO3TrajEstimator>(traj_->SO3Spline());

  addGyroscopeMeasurements(estimator_SO3);

  /// fix the initial pose of trajectory
  double weight_t0 = calib_param_manager->global_opt_gyro_weight;
  double t0 = traj_->SO3Spline()->MinTime();
  //Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  Eigen::AngleAxisd rotation_vector(0.0001, Eigen::Vector3d(0,0,1));
  Eigen::Quaterniond q0 = Eigen::Quaterniond (rotation_vector.matrix());
  auto m_q0 = std::make_shared<OrientationMeasurement>(t0, q0, weight_t0);
  estimator_SO3->AddMeasurement<OrientationMeasurement>(m_q0);

  ceres::Solver::Summary summary = estimator_SO3->Solve(30, false);
  std::cout << summary.BriefReport() << std::endl;
}

void TrajectoryManagerLVI::initialTrajWithIMUReadings() {
  // lidar_->set_relative_orientation(calib_param_manager->q_LtoI);
  // lidar_->set_relative_position(calib_param_manager->p_LinI);
  // lidar_->LockRelativeOrientation(false);
  // lidar_->LockRelativePosition(false);
  // if (opt_time_offset_ && time_offset_padding_ > 0) {
  //   lidar_->LockTimeOffset(false);
  // }else {
  //   lidar_->LockTimeOffset(true);
  // }
  imu_->LockGyroscopeBias(false);
  imu_->LockAccelerometerBias(false);

  std::shared_ptr<SplitTrajEstimator> estimator_split;
  estimator_split = std::make_shared<SplitTrajEstimator>(traj_);

  // add constraints  
  addGyroscopeMeasurements(estimator_split);
  addAccelerometerMeasurement(estimator_split);
  // addLidarPoses(estimator_split, lidar_start_time, lidar_poses);
  printErrorStatistics("Before optimization", true, true, false, true);
  calib_param_manager->showStates(false);
  ceres::Solver::Summary summary = estimator_split->Solve(50, false);
  std::cout << summary.BriefReport() << std::endl;
  printErrorStatistics("After optimization", true, true, false, true);
  
  calib_param_manager->set_p_LinI(lidar_->relative_position());
  calib_param_manager->set_q_LtoI(lidar_->relative_orientation());
  calib_param_manager->set_time_offset(lidar_->time_offset());
  calib_param_manager->set_gravity(imu_->refined_gravity());
  calib_param_manager->set_gyro_bias(imu_->gyroscope_bias());
  calib_param_manager->set_acce_bias(imu_->accelerometer_bias());
  calib_param_manager->showStates(false);
}

void TrajectoryManagerLVI::trajInitFromVisualFrames(
    const std::map<int64_t, std::shared_ptr<kontiki::sfm::View>>& cam_frames,
    bool opt_time_offset_ ){
  camera_->set_relative_orientation(calib_param_manager->q_CtoI);
  camera_->set_relative_position(calib_param_manager->p_CinI);
  camera_->LockRelativeOrientation(false);
  camera_->LockRelativePosition(false);
  lidar_->LockRelativeOrientation(true);
  lidar_->LockRelativePosition(true);
  if (opt_time_offset_ && time_offset_padding_ > 0) {
    camera_->LockTimeOffset(false);
  }else {
    camera_->LockTimeOffset(true);
  }
  imu_->LockGyroscopeBias(false);
  imu_->LockAccelerometerBias(false);

  std::shared_ptr<SplitTrajEstimator> estimator_split = 
    std::make_shared<SplitTrajEstimator>(traj_);

  // add constraints
  addGyroscopeMeasurements(estimator_split);
  addAccelerometerMeasurement(estimator_split);
  addVisualObservation(estimator_split, cam_frames);
  LOG(INFO)<<"Observation size: " << landmark_list_.size();
  printErrorStatistics("Before optimization");
  ceres::Solver::Summary summary = estimator_split->Solve(200, true);
  std::cout << summary.BriefReport() << std::endl;
  printErrorStatistics("After optimization");

  calib_param_manager->set_p_CinI(camera_->relative_position());
  calib_param_manager->set_q_CtoI(camera_->relative_orientation());
  calib_param_manager->set_time_offset(camera_->time_offset());
  calib_param_manager->set_gravity(imu_->refined_gravity());
  calib_param_manager->set_gyro_bias(imu_->gyroscope_bias());
  calib_param_manager->set_acce_bias(imu_->accelerometer_bias());
  calib_param_manager->showStates(true);
}

void TrajectoryManagerLVI::trajInitFromLVIdata(
    const std::map<int64_t, std::shared_ptr<kontiki::sfm::View>>& frames,
    SurfelAssociation::Ptr surfels_association,
    bool opt_time_offset_,  bool lock_traj_and_lidar){
  camera_->set_relative_orientation(calib_param_manager->q_CtoI);
  camera_->set_relative_position(calib_param_manager->p_CinI);
  camera_->LockRelativeOrientation(false);
  camera_->LockRelativePosition(false);
  lidar_->set_relative_orientation(calib_param_manager->q_LtoI);
  lidar_->set_relative_position(calib_param_manager->p_LinI);
  
  if(lock_traj_and_lidar){
    traj_->Lock(true);
    lidar_->LockRelativeOrientation(true);
    lidar_->LockRelativePosition(true);
  }else{
    traj_->Lock(false);
    lidar_->LockRelativeOrientation(false);
    lidar_->LockRelativePosition(false);
  }

  if (opt_time_offset_ && time_offset_padding_ > 0) {
    camera_->LockTimeOffset(false);
    lidar_->LockTimeOffset(false);
  }else {
    camera_->LockTimeOffset(true);
    lidar_->LockTimeOffset(true);
  }
  imu_->LockGyroscopeBias(false);
  imu_->LockAccelerometerBias(false);

  
  std::shared_ptr<SplitTrajEstimator> estimator_split = 
    std::make_shared<SplitTrajEstimator>(traj_);

  // add IMU data
  addGyroscopeMeasurements(estimator_split);
  addAccelerometerMeasurement(estimator_split);
  // add Lidar data
  addSurfMeasurement(estimator_split, surfels_association);
  // add Camera data
  addVisualObservation(estimator_split, frames);
  
  printErrorStatistics("Before optimization");
  ceres::Solver::Summary summary = estimator_split->Solve(80, true);
  std::cout << summary.BriefReport() << std::endl;
  printErrorStatistics("After optimization");

  calib_param_manager->set_p_CinI(camera_->relative_position());
  calib_param_manager->set_p_LinI(lidar_->relative_position());
  calib_param_manager->set_q_CtoI(camera_->relative_orientation());
  calib_param_manager->set_q_LtoI(lidar_->relative_orientation());
  calib_param_manager->set_time_offset(camera_->time_offset());
  calib_param_manager->set_gravity(imu_->refined_gravity());
  calib_param_manager->set_gyro_bias(imu_->gyroscope_bias());
  calib_param_manager->set_acce_bias(imu_->accelerometer_bias());
  calib_param_manager->showStates(true);
}

void TrajectoryManagerLVI::trajInitFromLVIdata(
    const std::map<int64_t, std::shared_ptr<kontiki::sfm::View>>& frames,
    SurfelAssociation::Ptr surfels_association,
    const std::map<kontiki::sfm::Landmark*, size_t>& lm_surfel,
    bool opt_time_offset_,  bool lock_traj_and_lidar){
  camera_->set_relative_orientation(calib_param_manager->q_CtoI);
  camera_->set_relative_position(calib_param_manager->p_CinI);
  camera_->LockRelativeOrientation(false);
  camera_->LockRelativePosition(false);
  lidar_->set_relative_orientation(calib_param_manager->q_LtoI);
  lidar_->set_relative_position(calib_param_manager->p_LinI);
  
  if(lock_traj_and_lidar){
    traj_->Lock(true);
    lidar_->LockRelativeOrientation(true);
    lidar_->LockRelativePosition(true);
  }else{
    traj_->Lock(false);
    lidar_->LockRelativeOrientation(false);
    lidar_->LockRelativePosition(false);
  }

  if (opt_time_offset_ && time_offset_padding_ > 0) {
    camera_->LockTimeOffset(false);
    lidar_->LockTimeOffset(false);
  }else {
    camera_->LockTimeOffset(true);
    lidar_->LockTimeOffset(true);
  }
  imu_->LockGyroscopeBias(false);
  imu_->LockAccelerometerBias(false);

  
  std::shared_ptr<SplitTrajEstimator> estimator_split = 
    std::make_shared<SplitTrajEstimator>(traj_);

  // add IMU data
  addGyroscopeMeasurements(estimator_split);
  addAccelerometerMeasurement(estimator_split);
  // add Lidar data
  addSurfMeasurement(estimator_split, surfels_association);
  // add Camera data
  addVisualObservation(estimator_split, frames);
  // add Camera-Lidar data
  addVisualLidarMeasurement(estimator_split, lm_surfel);
  
  printErrorStatistics("Before optimization");
  ceres::Solver::Summary summary = estimator_split->Solve(80, true);
  std::cout << summary.BriefReport() << std::endl;
  printErrorStatistics("After optimization");

  calib_param_manager->set_p_CinI(camera_->relative_position());
  calib_param_manager->set_p_LinI(lidar_->relative_position());
  calib_param_manager->set_q_CtoI(camera_->relative_orientation());
  calib_param_manager->set_q_LtoI(lidar_->relative_orientation());
  calib_param_manager->set_time_offset(camera_->time_offset());
  calib_param_manager->set_gravity(imu_->refined_gravity());
  calib_param_manager->set_gyro_bias(imu_->gyroscope_bias());
  calib_param_manager->set_acce_bias(imu_->accelerometer_bias());
  calib_param_manager->showStates(true);
}

void TrajectoryManagerLVI::trajInitFromLVIdata(const std::map<
    int64_t, std::shared_ptr<kontiki::sfm::View>>& cam_frames,
    const std::vector<std::pair<double, Eigen::Matrix4d>>& lidar_poses,
    double lidar_start_time,
    bool opt_time_offset_){
  camera_->set_relative_orientation(calib_param_manager->q_CtoI);
  camera_->set_relative_position(calib_param_manager->p_CinI);
  camera_->LockRelativeOrientation(false);
  camera_->LockRelativePosition(false);
  lidar_->set_relative_orientation(calib_param_manager->q_LtoI);
  lidar_->set_relative_position(calib_param_manager->p_LinI);
  lidar_->LockRelativeOrientation(false);
  lidar_->LockRelativePosition(false);
  if (opt_time_offset_ && time_offset_padding_ > 0) {
    camera_->LockTimeOffset(false);
    lidar_->LockTimeOffset(false);
  }else {
    camera_->LockTimeOffset(true);
    lidar_->LockTimeOffset(true);
  }
  imu_->LockGyroscopeBias(false);
  imu_->LockAccelerometerBias(false);
  
  

  std::shared_ptr<SplitTrajEstimator> estimator_split = 
    std::make_shared<SplitTrajEstimator>(traj_);

  // add IMU data
  addGyroscopeMeasurements(estimator_split);
  addAccelerometerMeasurement(estimator_split);
  // add Lidar data
  addLidarPoses(estimator_split, lidar_start_time, lidar_poses);
  // add Camera data
  addVisualObservation(estimator_split, cam_frames);
  
  printErrorStatistics("Before optimization");
  ceres::Solver::Summary summary = estimator_split->Solve(80, true);
  std::cout << summary.BriefReport() << std::endl;
  printErrorStatistics("After optimization");

  calib_param_manager->set_p_CinI(camera_->relative_position());
  calib_param_manager->set_p_LinI(lidar_->relative_position());
  calib_param_manager->set_q_CtoI(camera_->relative_orientation());
  calib_param_manager->set_q_LtoI(lidar_->relative_orientation());
  calib_param_manager->set_time_offset(camera_->time_offset());
  calib_param_manager->set_gravity(imu_->refined_gravity());
  calib_param_manager->set_gyro_bias(imu_->gyroscope_bias());
  calib_param_manager->set_acce_bias(imu_->accelerometer_bias());
  calib_param_manager->showStates(true);
}

void TrajectoryManagerLVI::trajInitFromSurfel(
        SurfelAssociation::Ptr surfels_association,
        bool opt_time_offset_) {
  lidar_->set_relative_orientation(calib_param_manager->q_LtoI);
  lidar_->set_relative_position(calib_param_manager->p_LinI);
  lidar_->LockRelativeOrientation(false);
  lidar_->LockRelativePosition(false);
  camera_->LockRelativeOrientation(true);
  camera_->LockRelativePosition(true);
  if (opt_time_offset_ && time_offset_padding_ > 0) {
    lidar_->LockTimeOffset(false);
  }
  else {
    lidar_->LockTimeOffset(true);
  }
  imu_->LockGyroscopeBias(false);
  imu_->LockAccelerometerBias(false);

  std::shared_ptr<SplitTrajEstimator> estimator_split;
  estimator_split = std::make_shared<SplitTrajEstimator>(traj_);

  // add constraints
  addGyroscopeMeasurements(estimator_split);
  addAccelerometerMeasurement(estimator_split);
  addSurfMeasurement(estimator_split, surfels_association);

  // addCallback(estimator_split);

  printErrorStatistics("Before optimization");
  ceres::Solver::Summary summary = estimator_split->Solve(30, false);
  std::cout << summary.BriefReport() << std::endl;
  printErrorStatistics("After optimization");

  calib_param_manager->set_p_LinI(lidar_->relative_position());
  calib_param_manager->set_q_LtoI(lidar_->relative_orientation());
  calib_param_manager->set_time_offset(lidar_->time_offset());
  calib_param_manager->set_gravity(imu_->refined_gravity());
  calib_param_manager->set_gyro_bias(imu_->gyroscope_bias());
  calib_param_manager->set_acce_bias(imu_->accelerometer_bias());
  calib_param_manager->showStates(true);
}

void TrajectoryManagerLVI::trajInitFromLidarPose(const std::vector<
    std::pair<double, Eigen::Matrix4d>>& lidar_poses, 
    double lidar_start_time, bool opt_time_offset_) {
  lidar_->set_relative_orientation(calib_param_manager->q_LtoI);
  lidar_->set_relative_position(calib_param_manager->p_LinI);
  lidar_->LockRelativeOrientation(false);
  lidar_->LockRelativePosition(false);
  if (opt_time_offset_ && time_offset_padding_ > 0) {
    lidar_->LockTimeOffset(false);
  }else {
    lidar_->LockTimeOffset(true);
  }
  imu_->LockGyroscopeBias(false);
  imu_->LockAccelerometerBias(false);

  std::shared_ptr<SplitTrajEstimator> estimator_split;
  estimator_split = std::make_shared<SplitTrajEstimator>(traj_);

  // add constraints  
  addGyroscopeMeasurements(estimator_split);
  addAccelerometerMeasurement(estimator_split);
  addLidarPoses(estimator_split, lidar_start_time, lidar_poses);
  printErrorStatistics("Before optimization", true, true, false, true);
  calib_param_manager->showStates(false);
  ceres::Solver::Summary summary = estimator_split->Solve(50, true);
  std::cout << summary.BriefReport() << std::endl;
  printErrorStatistics("After optimization", true, true, false, true);

  calib_param_manager->set_p_LinI(lidar_->relative_position());
  calib_param_manager->set_q_LtoI(lidar_->relative_orientation());
  calib_param_manager->set_time_offset(lidar_->time_offset());
  calib_param_manager->set_gravity(imu_->refined_gravity());
  calib_param_manager->set_gyro_bias(imu_->gyroscope_bias());
  calib_param_manager->set_acce_bias(imu_->accelerometer_bias());
  calib_param_manager->showStates(false);
}

bool TrajectoryManagerLVI::evaluateIMUPose(double imu_time, int flags,
                                        Result &result) const {
  if (traj_->MinTime() > imu_time || traj_->MaxTime() <= imu_time)
    return false;
  result = traj_->Evaluate(imu_time, flags);
  return true;
}

bool TrajectoryManagerLVI::evaluateLidarPose(double lidar_time,
                                          Eigen::Quaterniond &q_LtoG,
                                          Eigen::Vector3d &p_LinG) const {
  double traj_time = lidar_time + lidar_->time_offset();
  if (traj_->MinTime() > traj_time || traj_->MaxTime() <= traj_time)
    return false;
  Result result = traj_->Evaluate( traj_time, EvalOrientation | EvalPosition);
  q_LtoG = result->orientation * calib_param_manager->q_LtoI;
  p_LinG = result->orientation * calib_param_manager->p_LinI + result->position;
  return true;
}

bool TrajectoryManagerLVI::evaluateLidarRelativeRotation(double lidar_time1,
        double lidar_time2, Eigen::Quaterniond &q_L2toL1) const {
  assert(lidar_time1 <= lidar_time2
         && "[evaluateRelativeRotation] : lidar_time1 > lidar_time2");

  double traj_time1 = lidar_time1 + lidar_->time_offset();
  double traj_time2 = lidar_time2 + lidar_->time_offset();

  if (traj_->MinTime() > traj_time1 || traj_->MaxTime() <= traj_time2)
    return false;

  Result result1 = traj_->Evaluate(traj_time1, EvalOrientation);
  Result result2 = traj_->Evaluate(traj_time2, EvalOrientation);
  Eigen::Quaterniond q_I2toI1 = result1->orientation.conjugate()*result2->orientation;

  q_L2toL1 = calib_param_manager->q_LtoI.conjugate() * q_I2toI1 * calib_param_manager->q_LtoI;
  return true;
}


bool TrajectoryManagerLVI::evaluateCameraPose(double camera_time,
                                          Eigen::Quaterniond &q_CtoG,
                                          Eigen::Vector3d &p_CinG) const {
  double traj_time = camera_time + camera_->time_offset();
  if (traj_->MinTime() > traj_time || traj_->MaxTime() <= traj_time)
    return false;
  Result result = traj_->Evaluate(traj_time, EvalOrientation | EvalPosition);
  q_CtoG = result->orientation * calib_param_manager->q_CtoI;
  p_CinG = result->orientation * calib_param_manager->p_CinI + result->position;
  return true;
}

bool TrajectoryManagerLVI::evaluateCameraRelativeRotation(double camera_time1,
        double camera_time2, Eigen::Quaterniond &q_C2toC1) const {
  assert(camera_time1 <= camera_time2
         && "[evaluateRelativeRotation] : camera_time1 > camera_time2");

  double traj_time1 = camera_time1 + camera_->time_offset();
  double traj_time2 = camera_time2 + camera_->time_offset();

  if (traj_->MinTime() > traj_time1 || traj_->MaxTime() <= traj_time2)
    return false;

  Result result1 = traj_->Evaluate(traj_time1, EvalOrientation);
  Result result2 = traj_->Evaluate(traj_time2, EvalOrientation);
  Eigen::Quaterniond q_I2toI1 = 
    result1->orientation.conjugate() * result2->orientation;

  q_C2toC1 = calib_param_manager->q_CtoI.conjugate() 
    * q_I2toI1 * calib_param_manager->q_CtoI;
  return true;
}


template <typename TrajectoryModel>
void TrajectoryManagerLVI::addGyroscopeMeasurements(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator) {
  gyro_list_.clear();

  double weight = calib_param_manager->global_opt_gyro_weight;
  const double min_time = estimator->trajectory()->MinTime();
  const double max_time = estimator->trajectory()->MaxTime();
  // LOG(INFO)<< "IMU max_time - min_time: "<<max_time - min_time;
  size_t num = 0;
  for (const auto &v : imu_data_) {
    if ( min_time > v.timestamp || max_time <= v.timestamp) {
      continue;
    }
    // if(num++ % 4 != 0) continue;
    auto mg = std::make_shared<GyroMeasurement>(imu_, v.timestamp, v.gyro, weight);
    gyro_list_.push_back(mg);
    estimator->template AddMeasurement<GyroMeasurement>(mg);
  }
}

template <typename TrajectoryModel>
void TrajectoryManagerLVI::addAccelerometerMeasurement(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator) {
  accel_list_.clear();

  const double weight = calib_param_manager->global_opt_acce_weight;
  const double min_time = estimator->trajectory()->MinTime();
  const double max_time = estimator->trajectory()->MaxTime();
  // LOG(INFO)<< "IMU max_time - min_time: "<<max_time - min_time;
  size_t num = 0;
  for (auto const &v : imu_data_) {
    if ( min_time > v.timestamp || max_time <= v.timestamp) {
      continue;
    }
    // if(num++ % 4 != 0) continue;
    auto ma = std::make_shared<AccelMeasurement>(imu_, v.timestamp, v.accel, weight);
    accel_list_.push_back(ma);
    estimator->template AddMeasurement<AccelMeasurement>(ma);
  }
}

template <typename TrajectoryModel>
void TrajectoryManagerLVI::addVisualObservation(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
        const std::map<int64_t, std::shared_ptr<kontiki::sfm::View>>& frames){
  landmark_list_.clear();
  const double min_time = estimator->trajectory()->MinTime();
  const double max_time = estimator->trajectory()->MaxTime();
  const double weight = calib_param_manager->global_opt_cam_weight;
  for(auto iter = frames.begin(); iter != frames.end(); iter++){
    if ( min_time > iter->second->t0() || max_time <= iter->second->t0()) {
      continue;
    }
    for(const auto& obs : iter->second->observations()){
      if(obs->landmark()->observations().size() <= 5) {
        continue;
      }
      if(obs->landmark()->inverse_depth() <= 0) {
        continue;
      }
      auto ms_obs = std::make_shared<CameraMeasurement>(camera_, obs, weight);
      landmark_list_.push_back(ms_obs);
      estimator->template AddMeasurement<CameraMeasurement>(ms_obs);
    }
  }
}

template <typename TrajectoryModel>
void TrajectoryManagerLVI::addLidarPoses(
      std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
      double lidar_start_time,
      const std::vector<std::pair<double, Eigen::Matrix4d>>& lidar_poses){
  const double pos_weight = calib_param_manager->global_opt_pos_weight;//TODO
  const double rot_weight = calib_param_manager->global_opt_rot_weight;//TODO
  const double min_time = estimator->trajectory()->MinTime();
  const double max_time = estimator->trajectory()->MaxTime();
  lori_list_.clear();
  lpos_list_.clear();
  for(const auto& pr: lidar_poses){
    if( min_time > pr.first || max_time <= pr.first) continue;
    if( min_time > lidar_start_time || max_time <= lidar_start_time) continue;
    auto pos = std::make_shared<LidarPositionMeasurement>(
      lidar_, pr.first, lidar_start_time, pos_weight, 
      pr.second.block<3, 1>(0, 3),
      Eigen::Quaterniond(pr.second.block<3,3>(0,0)));  
    lpos_list_.push_back(pos);
    estimator->template AddMeasurement<LidarPositionMeasurement>(pos);

    // auto rot = std::make_shared<LidarOrientationMeasurement>(
    //   lidar_, pr.first, lidar_start_time, rot_weight, 
    //   Eigen::Quaterniond(pr.second.block<3,3>(0,0)));  
    // lori_list_.push_back(rot);
    // estimator->template AddMeasurement<LidarOrientationMeasurement>(rot);
  }
} 

template <typename TrajectoryModel>
void TrajectoryManagerLVI::addSurfMeasurement(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
        const SurfelAssociation::Ptr surfel_association) {
  const double weight = calib_param_manager->global_opt_lidar_weight;
  surfelpoint_list_.clear();
  closest_point_vec_.clear();
  for (auto const& v: surfel_association->get_surfel_planes()) {
    closest_point_vec_.push_back(v.Pi);
  }

  map_time_ = surfel_association->get_maptime();
  for (auto const &spoint : surfel_association->get_surfel_points()) {
    double time = spoint.timestamp;
    size_t plane_id = spoint.plane_id;

    auto msp = std::make_shared<SurfMeasurement> (lidar_, spoint.point,
                                                  closest_point_vec_.at(plane_id).data(), time, map_time_, 5.0, weight);
    surfelpoint_list_.push_back(msp);
    estimator->template AddMeasurement<SurfMeasurement>(msp);
  }
}

template <typename TrajectoryModel>
void TrajectoryManagerLVI::addVisualLidarMeasurement(
    std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
    const std::map<kontiki::sfm::Landmark*, size_t>& lm_surfel) {
  const double weight = calib_param_manager->global_opt_visual_surfel_weight;//todo
  surf_landmark_list_.clear();
  // added in addSurfMeasurement
  // closest_point_vec_.clear();
  // for (auto const& v: surfel_association->get_surfel_planes()) {
  //   closest_point_vec_.push_back(v.Pi);
  // }

  for (const auto &ls : lm_surfel) {
    double time = ls.first->reference()->view()->t0();
    size_t plane_id = ls.second;

    auto lsm = std::make_shared<CameraSurfMeasurement>(
      camera_, lidar_, ls.first, closest_point_vec_.at(plane_id).data(), 
      time, map_time_, 5.0, weight);
    surf_landmark_list_.push_back(lsm);
    estimator->template AddMeasurement<CameraSurfMeasurement>(lsm);
  }
}

template <typename TrajectoryModel>
void TrajectoryManagerLVI::addCallback(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator) {
  // Add callback for debug
  std::unique_ptr<CheckStateCallback> cb  = std::make_unique<CheckStateCallback>();
  cb->addCheckState("q_LtoI     :", 4, lidar_->relative_orientation().coeffs().data());
  cb->addCheckState("p_LinI     :", 3, lidar_->relative_position().data());
  cb->addCheckState("time_offset:", 1, &lidar_->time_offset());
  cb->addCheckState("g_roll     :", 1, &imu_->gravity_orientation_roll());
  cb->addCheckState("g_pitch    :", 1, &imu_->gravity_orientation_pitch());
  estimator->AddCallback(std::move(cb), true);
}

void TrajectoryManagerLVI::printErrorStatistics(
    const std::string& intro, bool show_gyro, bool show_accel, 
    bool show_lidar,bool show_lidar_pose, bool show_camera) {
  std::cout << "\n============== " << intro << " ================" << std::endl;

  if (show_gyro && !gyro_list_.empty()) {
    Eigen::Vector3d error_sum;
    for(auto const& m : gyro_list_) {
      error_sum += m->ErrorRaw<SplitTrajectory> (*traj_).cwiseAbs();
    }
    err_gyr_[0] = gyro_list_.size();
    err_gyr_.segment<3>(1) = error_sum / gyro_list_.size();
    std::cout << "[Gyro]  Error size, average: " << gyro_list_.size()
              << "; " << (error_sum/gyro_list_.size()).transpose() << std::endl;
  }

  if (show_accel && !accel_list_.empty()) {
    Eigen::Vector3d error_sum;
    for(auto const& m : accel_list_) {
      error_sum += m->ErrorRaw<SplitTrajectory> (*traj_).cwiseAbs();
    }
    err_acc_[0] = accel_list_.size();
    err_acc_.segment<3>(1) = error_sum/accel_list_.size();
    std::cout << "[Accel] Error size, average: " << accel_list_.size()
              << ";  " << (error_sum/accel_list_.size()).transpose() << std::endl;
  }

  if (show_lidar && !surfelpoint_list_.empty()) {
    Eigen::Matrix<double,1,1>  error_sum;
    for (auto const &m : surfelpoint_list_) {
      error_sum += m->point2plane<SplitTrajectory>(*traj_).cwiseAbs();
    }
    err_surfel_[0] = surfelpoint_list_.size();
    err_surfel_.segment<1>(1) = error_sum/surfelpoint_list_.size();
    std::cout << "[LiDAR] Error size, average: " << surfelpoint_list_.size()
              << "; " << (error_sum/surfelpoint_list_.size()).transpose() << std::endl;
  }


  if (show_lidar_pose) {
    if(!lpos_list_.empty()){
      Eigen::Vector3d error_sum_pos;
      // Eigen::Matrix<double, 1,1> error_sum_pos;
      for(auto const& m : lpos_list_) {
        // error_sum_pos += m->Error<SplitTrajectory> (*traj_, *lidar_).cwiseAbs();
        error_sum_pos += m->Error<SplitTrajectory> (*traj_, *lidar_);
      }
      std::cout << "[LiDAR Pos] Error size, average: " << lpos_list_.size()
                << ";  " << (error_sum_pos/lpos_list_.size()).transpose() << std::endl;
    }
    
    if(!lori_list_.empty()){
      double error_sum_ori;
      const double scale = 180.0 / M_PI;
      for(auto const& m : lori_list_) {
        error_sum_ori += m->Error<SplitTrajectory> (*traj_, *lidar_);
      }
      std::cout << "[LiDAR Ori] Error size, average: " << lori_list_.size()
                << ";  " << scale*(error_sum_ori/lori_list_.size()) << std::endl;
    }
  }
  
  if(show_camera && !landmark_list_.empty()){
    Eigen::Vector2d error_sum_uv;
    for(auto const& m : landmark_list_) {
      error_sum_uv += m->Error<SplitTrajectory> (*traj_);
    }

    err_cam_[0] = landmark_list_.size();
    err_cam_.segment<2>(1) = error_sum_uv/landmark_list_.size();
    
   
    std::cout << "[CAMERA] Error size, average: " << landmark_list_.size()
              << "; " << (error_sum_uv/landmark_list_.size()).transpose() << std::endl;
  }
  std::cout << std::endl;
}

}