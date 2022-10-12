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
#ifndef CALIB_HELPER_H
#define CALIB_HELPER_H

#include <core/inertial_initializer.h>
#include <core/lidar_odometry.h>
#include <core/surfel_association.h>
#include <utils/dataset_reader.h>
#include <core/scan_undistortion.h>

namespace licalib {

class CalibrHelperLVI {
public:
  enum CalibStep {
    Error = 0,
    Start,
    InitializationDone,
    DataAssociationDone,
    BatchOptimizationDone,
    RefineDone
  };
  explicit CalibrHelperLVI(ros::NodeHandle& nh);

  bool createCacheFolder(const std::string& bag_path);
  
  bool Initialization();

  

  void saveCalibResult(const std::string& calib_result_file) const;

  void saveMap() const;
  
  TrajectoryManagerLVI::Ptr GetTrajectoryManager(){
    return traj_manager_;
  }

  std::shared_ptr<IO::LioDataset> GetDataReader(){
    return dataset_reader_;
  }
protected:

  /// dataset
  std::string cache_path_;
  std::string topic_imu_;
  std::string bag_path_;

  /// optimization
  CalibStep calib_step_;
  int iteration_step_;
  bool opt_time_offset_;

  /// lidar odometry
  double map_time_;
  double ndt_resolution_;
  double scan4map_time_;

  /// data association
  double associated_radius_;
  double plane_lambda_;

  std::shared_ptr<IO::LioDataset> dataset_reader_;
  InertialInitializer::Ptr rotation_initializer_;
  TrajectoryManagerLVI::Ptr traj_manager_;
  LiDAROdometry::Ptr lidar_odom_;
  SurfelAssociation::Ptr surfel_association_;

  ScanUndistortion::Ptr scan_undistortion_;
  CameraIntrinsic camera_params_;
};

}

#endif
