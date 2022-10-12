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
#include <core/calib_helper_lvi.h>
#include <core/scan_undistortion.h>
#include <utils/tic_toc.h>

#include <boost/filesystem.hpp>
#include <memory>
#include <sstream>
#include <glog/logging.h>

namespace licalib {

CalibrHelperLVI::CalibrHelperLVI(ros::NodeHandle& nh){}

bool CalibrHelperLVI::createCacheFolder(const std::string& bag_path) {
  boost::filesystem::path p(bag_path);
  if (p.extension() != ".bag") {
    return false;
  }
  cache_path_ = p.parent_path().string() + "/" + p.stem().string();
  boost::filesystem::create_directory(cache_path_);
  return true;
}

bool CalibrHelperLVI::Initialization() {
  if (Start != calib_step_) {
    ROS_WARN("[Initialization] Need status: Start.");
    return false;
  }
  
  for (const auto& imu_data: dataset_reader_->get_imu_data()) {
    traj_manager_->feedIMUData(imu_data);
  }
  traj_manager_->initialSO3TrajWithGyro();

  for(const TPointCloud& raw_scan: dataset_reader_->get_scan_data()) {
    VPointCloud::Ptr cloud(new VPointCloud);
    TPointCloud2VPointCloud(raw_scan.makeShared(), cloud);
    double scan_timestamp = pcl_conversions::fromPCL(raw_scan.header.stamp).toSec();

    lidar_odom_->feedScan(scan_timestamp, cloud);

    if (lidar_odom_->get_odom_data().size() < 30
        || (lidar_odom_->get_odom_data().size() % 10 != 0))
      continue;
    if (rotation_initializer_->EstimateRotation(traj_manager_,
                                                lidar_odom_->get_odom_data())) {
      Eigen::Quaterniond qItoLidar = rotation_initializer_->getQ_ItoS();
      traj_manager_->getCalibParamManager()->set_q_LtoI(qItoLidar.conjugate());

      Eigen::Vector3d euler_ItoL = qItoLidar.toRotationMatrix().eulerAngles(0,1,2);
      std::cout << "[Initialization] Done. Euler_ItoL initial degree: "
                << (euler_ItoL*180.0/M_PI).transpose() << std::endl;
      
      double roll = euler_ItoL[0];
      double pitch = euler_ItoL[1];
      double yaw = euler_ItoL[2];
      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond q_ItoL = rollAngle * pitchAngle * yawAngle;
      Eigen::Matrix3d r_ItoL = q_ItoL.toRotationMatrix();
      std::cout << "Rot_ItoL: \n"
                << r_ItoL
                << std::endl;

      calib_step_ = InitializationDone;
      break;
    }
  }
  if (calib_step_ != InitializationDone){
    ROS_WARN("[Initialization] fails.");
    return false;
  }
  return true;
    
}


void CalibrHelperLVI::saveCalibResult(const std::string& calib_result_file) const {
  if (!boost::filesystem::exists(calib_result_file)) {
    std::ofstream outfile;
    outfile.open(calib_result_file, std::ios::app);
    outfile << "bag_path" << ","
            << "imu_topic" << "," << "map_time" << "," << "iteration_step" << ","
            << "p_IinL.x" << "," << "p_IinL.y" << "," << "p_IinL.z" << ","
            << "q_ItoL.x" << "," << "q_ItoL.y" << "," << "q_ItoL" << ","
            << "q_ItoL.w" << ","
            << "time_offset" << ","
            << "gravity.x" << "," << "gravity.y" << "," << "gravity.z" << ","
            << "gyro_bias.x" << "," << "gyro_bias.y" << "," <<"gyro_bias.z" << ","
            << "acce_bias.z" << "," << "acce_bias.y" << "," <<"acce_bias.z" << "\n";
    outfile.close();
  }

  std::stringstream ss;
  ss << bag_path_;
  ss << "," << topic_imu_;
  ss << "," << map_time_;
  ss << "," << iteration_step_;
  std::string info;
  ss >> info;

  traj_manager_->getCalibParamManager()->save_result(calib_result_file, info);
}

void CalibrHelperLVI::saveMap() const {
  if (calib_step_ <= Start)
    return;
  std::string NDT_target_map_path = cache_path_ + "/NDT_target_map.pcd";
  lidar_odom_->saveTargetMap(NDT_target_map_path);

  std::string surfel_map_path = cache_path_ + "/surfel_map.pcd";
  surfel_association_->saveSurfelsMap(surfel_map_path);

  if (RefineDone == calib_step_) {
    std::string refined_map_path = cache_path_ + "/refined_map.pcd";
    std::cout << "Save refined map to " << refined_map_path << "; size: "
              << scan_undistortion_->get_map_cloud()->size() << std::endl;
    pcl::io::savePCDFileASCII(refined_map_path, *scan_undistortion_->get_map_cloud());
  }
}

}
