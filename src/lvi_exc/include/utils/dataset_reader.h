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
#ifndef DATASET_READER_H
#define DATASET_READER_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>

#include <fstream>
#include <Eigen/Core>
#include <utils/eigen_utils.hpp>
#include <utils/vlp_common.h>

namespace licalib {
namespace IO {


struct IMUData {
  double timestamp;
  Eigen::Matrix<double, 3, 1> gyro;
  Eigen::Matrix<double, 3, 1> accel;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ImageData {
  double timestamp;
  cv::Mat image;
};

struct PoseData {
  double timestamp;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


enum LidarModelType {
    VLP_16,
    VLP_16_SIMU,
    RS_16,
    VLP_16_SIM,
};

class LioDataset {
private:
  std::shared_ptr<rosbag::Bag> bag_;

  Eigen::aligned_vector<IMUData> imu_data_;

  Eigen::aligned_vector<TPointCloud> scan_data_;
  std::vector<ImageData> image_data_;
  std::vector<double> scan_timestamps_;

  double start_time_;
  double end_time_;

  VelodyneCorrection::Ptr p_LidarConvert_;

  LidarModelType lidar_model_;

public:
  LioDataset(LidarModelType lidar_model) : lidar_model_(lidar_model) {

  }

  void init() {
    switch (lidar_model_) {
      case LidarModelType::VLP_16:
        p_LidarConvert_ = VelodyneCorrection::Ptr(
                new VelodyneCorrection(VelodyneCorrection::ModelType::VLP_16));
        break;
      case LidarModelType::VLP_16_SIMU:
        p_LidarConvert_ = VelodyneCorrection::Ptr(
                new VelodyneCorrection(VelodyneCorrection::ModelType::VLP_16));
        break;
      case LidarModelType::RS_16:
        //
        break;
      case LidarModelType::VLP_16_SIM:
        //
        break;
      default:
        std::cout << "LiDAR model " << lidar_model_
                  << " not support yet." << std::endl;
    }
  }

  bool read(const std::string path,
            const std::string imu_topic,
            const std::string lidar_topic,
            const std::string image_topic,
            const double bag_start = -1.0,
            const double bag_durr = -1.0) {

    bag_.reset(new rosbag::Bag);
    bag_->open(path, rosbag::bagmode::Read);

    init();

    rosbag::View view;
    {
      std::vector<std::string> topics;
      topics.push_back(imu_topic);
      topics.push_back(lidar_topic);
      topics.push_back(image_topic);

      rosbag::View view_full;
      view_full.addQuery(*bag_);
      ros::Time time_init = view_full.getBeginTime();
      time_init += ros::Duration(bag_start);
      ros::Time time_finish = (bag_durr < 0)?
        view_full.getEndTime() : time_init + ros::Duration(bag_durr);
      view.addQuery(
        *bag_, rosbag::TopicQuery(topics), time_init, time_finish);
    }

    for (rosbag::MessageInstance const m : view) {
      const std::string &topic = m.getTopic();

      if (lidar_topic == topic) {
        TPointCloud pointcloud;
        double timestamp = 0;

        if (m.getDataType() == std::string("velodyne_msgs/VelodyneScan")) {
          velodyne_msgs::VelodyneScan::ConstPtr vlp_msg =
                  m.instantiate<velodyne_msgs::VelodyneScan>();
          timestamp = vlp_msg->header.stamp.toSec();
          p_LidarConvert_->unpack_scan(vlp_msg, pointcloud);
        }
        if (m.getDataType() == std::string("sensor_msgs/PointCloud2")) {
          sensor_msgs::PointCloud2::ConstPtr scan_msg =
                  m.instantiate<sensor_msgs::PointCloud2>();
          if(lidar_model_ == RS_16){
            pcl::PointCloud<RsPointXYZIRT> pcl_point_cloud;
            pcl::fromROSMsg(*scan_msg, pcl_point_cloud);
            RsPointCloud2TPointCloud(pcl_point_cloud, pointcloud);
            //RS LiDAR的ROS时间戳为最后一个点的采样时间，更改到第一个采样点下
            std_msgs::Header ros_header = scan_msg->header;
            if(!pointcloud.points.empty()){
              ros_header.stamp = ros::Time(pointcloud.points.front().timestamp);
            }
            pcl::PCLHeader pcl_header = pcl_conversions::toPCL(ros_header);
            pointcloud.header = pcl_header; 
            timestamp = pcl_conversions::fromPCL(pointcloud.header.stamp).toSec();
          }else if(lidar_model_ == VLP_16_SIM){
            pcl::PointCloud<RsPointXYZIRT> pcl_point_cloud;
            pcl::fromROSMsg(*scan_msg, pcl_point_cloud);
            pcl_point_cloud.is_dense = false;
            RsPointCloud2TPointCloud(pcl_point_cloud, pointcloud);
            timestamp = scan_msg->header.stamp.toSec();
          }else{
            p_LidarConvert_->unpack_scan(scan_msg, pointcloud);
            timestamp = scan_msg->header.stamp.toSec();
          }
        }

        scan_data_.emplace_back(pointcloud);
        scan_timestamps_.emplace_back(timestamp);
      }

      if (imu_topic == topic) {
        sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
        double time = imu_msg->header.stamp.toSec();

        imu_data_.emplace_back();
        imu_data_.back().timestamp = time;
        imu_data_.back().accel = Eigen::Vector3d(
                imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
                imu_msg->linear_acceleration.z);
        imu_data_.back().gyro = Eigen::Vector3d(
                imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
                imu_msg->angular_velocity.z);
      }

      if (image_topic == topic) {
        sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
        cv_bridge::CvImageConstPtr cv_ptr;
        try{
          cv_ptr = cv_bridge::toCvShare(img_msg);
        }catch (cv_bridge::Exception& e){
          std::cerr<<"Read image from bag error!\n";
          return false;
        }

        image_data_.emplace_back();
        image_data_.back().timestamp = img_msg->header.stamp.toSec();
        cv_ptr->image.copyTo(image_data_.back().image);
      }
    }
    
    // start_time_ = std::min(scan_timestamps_.front(), imu_data_.front().timestamp);
    // end_time_ = std::max(scan_timestamps_.back(), imu_data_.back().timestamp);
    std::cout << lidar_topic << ": " << scan_data_.size() << std::endl;
    std::cout << imu_topic << ": " << imu_data_.size() << std::endl;
    std::cout << image_topic << ": " << image_data_.size() << std::endl;
  }


  /// Select the overlap of the dataset
  ///     ----------------------- ...... -----------------------------> t
  ///     |       |        |                    |         |        |
  ///   scan_0  IMU_0    scan_1               scan_N-1  IMU_N    scan_N
  ///
  /// selected data [scan_0, scan_N-1],[IMU_0, IMU_N]
  /// time  [scan_0.t, scan_N.t)
  void adjustDataset() {
    assert(imu_data.size() > 0 && "No IMU data. Check your bag and imu topic");
    assert(scan_data.size() > 0 && "No scan data. Check your bag and lidar topic");

    assert(scan_timestamps.front() < imu_data.back().timestamp
           && scan_timestamps.back() > imu_data.front().timestamp
           && "Unvalid dataset. Check your dataset.. ");

    if (scan_timestamps_.front() > imu_data_.front().timestamp) {
      start_time_ = scan_timestamps_.front();
      while (imu_data_.front().timestamp < start_time_)
        imu_data_.erase(imu_data_.begin());
    } else {
      while ((*std::next(scan_timestamps_.begin())) < start_time_) {
        scan_data_.erase(scan_data_.begin());
        scan_timestamps_.erase(scan_timestamps_.begin());
      }
      start_time_ = scan_timestamps_.front();
    }

    if (scan_timestamps_.front() > image_data_.front().timestamp) {
      start_time_ = scan_timestamps_.front();
      while (image_data_.front().timestamp < start_time_)
        image_data_.erase(image_data_.begin());
    }

    end_time_ = std::min(scan_timestamps_.back(), imu_data_.back().timestamp);

    while (imu_data_.back().timestamp > end_time_)
      imu_data_.pop_back();

    while (image_data_.back().timestamp > end_time_)
      image_data_.pop_back();

    while (scan_timestamps_.back() >= end_time_) {
      scan_data_.pop_back();
      scan_timestamps_.pop_back();
    }
    //std::cout<<"after adjust --> imu size : "<< imu_data_.size() << std::endl;
  }

  // void reset() { data_.reset(); }

  double get_start_time() const {
    return start_time_;
  }

  double get_end_time() const {
    return end_time_;
  }

  const std::vector<double> &get_scan_timestamps() const {
    return scan_timestamps_;
  }

  const Eigen::aligned_vector<IMUData> &get_imu_data() const {
    return imu_data_;
  }
  const Eigen::aligned_vector<TPointCloud> &get_scan_data() const {
    return scan_data_;
  }
  const std::vector<ImageData> &get_image_data() const {
    return image_data_;
  }

};

}
}

#endif // DATASET_READER_H
