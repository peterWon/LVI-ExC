#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <random>
#include <cmath>  
//ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include "sensor_msgs/PointCloud2.h"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>


#include <Eigen/Core>
#include <Eigen/Dense>
#include <glog/logging.h>

#include "vi_init/integration_base.h"
#include "vi_init/feature_tracker.h"
#include "vi_init/feature_manager.h"
#include "vi_init/sfm.h"
#include "vi_init/solve_5pts.h"
#include "vi_init/utility.h"
#include "vi_init/initial_ex_rotation.h"
#include "vi_init/initial_alignment.h"

//LI-Calib
#include "utils/dataset_reader.h"
#include "core/trajectory_manager_lvi.h"
#include "core/calib_helper_lvi.h"

#include "kontiki/sfm/sfm.h"

#include <chrono>   
using namespace std;
using namespace chrono;

#include <sstream>

namespace {
std::vector<std::string> SplitString(const std::string& input,
                                     const char delimiter) {
  std::istringstream stream(input);
  std::string token;
  std::vector<std::string> tokens;
  while (std::getline(stream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
};
/* 
// yaw (Z), pitch (Y), roll (X)
Eigen::Quaterniond yprToQuaternion(double yaw, double pitch, double roll) {
  // Abbreviations for the various angular functions
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Quaterniond q;
  q.w() = cy * cp * cr + sy * sp * sr;
  q.x() = cy * cp * sr - sy * sp * cr;
  q.y() = sy * cp * sr + cy * sp * cr;
  q.z() = sy * cp * cr - cy * sp * sr;

  return q;
}
 */
} 

namespace licalib{
class LIinitializer: public licalib::CalibrHelperLVI {
public:
  struct IntegrationFrame{
  public:
    IntegrationFrame(){}
    int64_t timestamp = 0;
    std::shared_ptr<IntegrationBase> integrator = nullptr;
    Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();
  };

  LIinitializer(ros::NodeHandle& nh): CalibrHelperLVI(nh){
    imu_noise_.ACC_N = 1e-1;
    imu_noise_.ACC_W = 1e-3;
    imu_noise_.GYR_N = 1e-1;
    imu_noise_.GYR_W = 1e-3;


    std::string topic_lidar;
    std::string topic_image;
    double bag_start, bag_durr;
    double scan4map;
    // double knot_distance;
    // double time_offset_padding;
    
    calib_step_ = Start;
    iteration_step_ = 0;
    opt_time_offset_ = false;
    plane_lambda_ = 0.6;
    ndt_resolution_ = 0.5;
    associated_radius_ = 0.05;
    nh.param<std::string>("bag_filename", bag_path_, "V1_01_easy.bag");
    nh.param<bool>("save_rendered_map", save_rendered_map_, false);
    nh.param<std::string>("pcd_outpath", pcd_outpath_, "");
    nh.param<std::string>("calib_result_outpath", calib_result_outpath_, "");
    nh.param<std::string>("imu_topic", topic_imu_, "/imu0");
    nh.param<std::string>("lidar_topic", topic_lidar, "/velodyne_packets");
    nh.param<std::string>("image_topic", topic_image, "/image_raw_gray");
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    nh.param<double>("scan4map", scan4map, 15);
    nh.param<double>("ndtResolution", ndt_resolution_, 0.5);
    nh.param<double>("associated_radius", associated_radius_, 0.05);
    nh.param<int>("border_filter_uv", border_filter_uv_, 10);

    nh.param<bool>("simulation", simulation_, false);

    std::string bag_file_path = "";
    std::string orb_folder = "";
    std::string loam_folder = "";
    CHECK(nh.getParam("bag_filename", bag_file_path)) 
      << "'bag_filename' must be assigned!";
    CHECK(nh.getParam("orb_folder", orb_folder)) 
      << "'orb_folder' must be assigned!";
    CHECK(nh.getParam("loam_folder", loam_folder)) 
      << "'loam_folder' must be assigned!";
    std::string file_path_noext = 
      bag_file_path.substr(0, bag_file_path.find_last_of("."));
    std::string file_dir = 
      file_path_noext.substr(0, bag_file_path.find_last_of("/")); 
    bag_name_ = 
      file_path_noext.substr(bag_file_path.find_last_of("/")); 
    
    // nh.param<double>("time_offset_padding", time_offset_padding, 0.015);
    // nh.param<double>("knot_distance", knot_distance, 0.02);
    
    nh.param<double>("time_offset_padding", time_padding_, 0.015);
    nh.param<double>("knot_distance", knot_space_, 0.02);
    nh.param<bool>("optimize_time_offset", optimize_time_offset_, false);
    nh.param<bool>("lock_traj_lidar_in_2nd_stage", 
      lock_traj_lidar_in_2nd_stage_, false);
    nh.param<bool>("lock_traj_lidar_in_3rd_stage", 
      lock_traj_lidar_in_3rd_stage_, false);

    {
      std::string lidar_model;
      nh.param<std::string>("LidarModel", lidar_model, "RS_16");
      IO::LidarModelType lidar_model_type = IO::LidarModelType::RS_16;
      if (lidar_model == "RS_16") {
        lidar_model_type = IO::LidarModelType::RS_16;
      } else if(lidar_model == "VLP_16_SIM"){
        lidar_model_type = IO::LidarModelType::VLP_16_SIM;
      } else {
        calib_step_ = Error;
        ROS_WARN("LiDAR model %s not support yet.", lidar_model.c_str());
      }
      /// read dataset
      std::cout << "\nLoad dataset from " << bag_path_ << std::endl;
      dataset_reader_ = std::make_shared<IO::LioDataset>(lidar_model_type);
      dataset_reader_->read(
        bag_path_, topic_imu_, topic_lidar, topic_image, bag_start, bag_durr);
      //[t_scan_0, t_scan_N]
      dataset_reader_->adjustDataset();
    }
    map_time_ = dataset_reader_->get_start_time();
    scan4map_time_ = map_time_ + scan4map;
    double end_time = dataset_reader_->get_end_time();
    
    //currently only the pinhole model is supported
    //must be set
    CHECK(nh.getParam("image_height", camera_params_.row));
    CHECK(nh.getParam("image_width", camera_params_.col));
    CHECK(nh.getParam("readout", camera_params_.readout));
    CHECK(nh.getParam("projection_parameters/cx", camera_params_.cx));
    CHECK(nh.getParam("projection_parameters/cy", camera_params_.cy));
    CHECK(nh.getParam("projection_parameters/fx", camera_params_.fx));
    CHECK(nh.getParam("projection_parameters/fy", camera_params_.fy));
    //optional
    nh.getParam("distortion_parameters/k1", camera_params_.k1);
    nh.getParam("distortion_parameters/k2", camera_params_.k2);
    nh.getParam("distortion_parameters/p1", camera_params_.p1);
    nh.getParam("distortion_parameters/p2", camera_params_.p2);
    nh.getParam("distortion_parameters/k3", camera_params_.k3);
    
    LOG(INFO)<<"Using camera intrinsics: ";
    LOG(INFO)<<camera_params_.cx<<","<<camera_params_.cy<<","<<camera_params_.fx<<","<<camera_params_.fy;
    LOG(INFO)<<camera_params_.k1<<","<<camera_params_.k2<<","<<camera_params_.p1<<","<<camera_params_.p2;
    start_time_ = map_time_;
    end_time_ = end_time; 

    map_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 1);
  }

  ~LIinitializer(){}
  
  void LoadOrbResults(const std::string& orb_res_path);
  void NdtMapping();
  void ReadPoseGT(const std::string& pose_file_path);//use ndt instead？
  void InitVariables(){
    LOG(INFO)<<"end_time_ - start_time_: "<<end_time_ - start_time_;
    traj_manager_ = std::make_shared<TrajectoryManagerLVI>(
      camera_params_, start_time_, end_time_, knot_space_, time_padding_);

    scan_undistortion_ = std::make_shared<ScanUndistortion>(
            traj_manager_, dataset_reader_);

    lidar_odom_ = std::make_shared<LiDAROdometry>(ndt_resolution_);

    rotation_initializer_ = std::make_shared<InertialInitializer>();

    surfel_association_ = std::make_shared<SurfelAssociation>(
            associated_radius_, plane_lambda_);
  }

  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void RenderMap();

  void ComputeIntegrationForFrames(){
    ComputeIntegrationForFrames(integration_frames_cam_, imu_cache_cam_);
    ComputeIntegrationForFrames(integration_frames_lidar_, imu_cache_lidar_);
  }
  void EstimateInitExtrinsicCI();
  void EstimateInitExtrinsicLI();
  void CIoptimize();
  void LIoptimize();
  void LCIoptimize();
  void SaveOptimizedPCDMap(const std::string filepath);
  void ReprojectPointCloudToImage();
  void PublishMapCloud();
private:
  bool findAssociatedPose(
    int idx_in_data_reader, double timestamp, Eigen::Matrix4d& loam_pose);
  void Mapping(bool relocalization = false);
  void DataAssociation();

  void BatchOptimization();

  void Refinement();
  
  void ComputeIntegrationForFrames(
    std::deque<IntegrationFrame>& integration_frames,
    std::deque<sensor_msgs::Imu>& imu_cache);
  void SetCalibWeights(int& skip_frame);
  bool CheckObservibility();
  void PopOldIMU(const double stamp, std::deque<sensor_msgs::Imu>& imu_cache);
  void RemoveOverTimeFrames(
    const std::deque<sensor_msgs::Imu>& imu_cache,
    std::deque<IntegrationFrame>& integration_frames);
  
  void PublishTrajectory();

  int64_t frame_index_ = 0;
  IMUNoise imu_noise_;
  bool imu_recieved_ = false;
  bool img_recieved_ = false;
  double last_imu_time_ = -1;
  

  std::deque<sensor_msgs::Imu> imu_cache_cam_;
  std::deque<sensor_msgs::Imu> imu_cache_lidar_;
  std::deque<licalib::IO::IMUData> imu_quene_for_traj_={};

  InitialEXRotation rot_estimator_;
  bool init_rot_est_ok_cam_ = false;
  bool init_rot_est_ok_lidar_ = false;
  bool init_cam_ok_ = false;
  bool init_lidar_ok_ = false;
  Matrix3d R_I_L_;
  Vector3d T_I_L_;
  Matrix3d R_I_C_;
  Vector3d T_I_C_;
  Vector4d g_I_t_cam_;
  Vector4d g_I_t_lidar_;
  
  std::map<int64_t, std::shared_ptr<kontiki::sfm::Landmark>> landmark_db_={};
  std::map<int64_t, std::shared_ptr<kontiki::sfm::View>> views_db_={};
  std::map<int64_t, std::map<int64_t, Eigen::Vector2d>> uv_points = {}; 
  
  //for initialization usage
  std::deque<IntegrationFrame> integration_frames_cam_;
  std::deque<IntegrationFrame> integration_frames_lidar_;
  std::map<int64_t, Eigen::Matrix4d> loam_poses_map_;//for initial mapping
  std::vector<std::pair<double, Eigen::Matrix4d>> loam_poses_;//for initial mapping

  double last_img_time_ = -1;

  double start_time_ = -1.0;
  double end_time_ = -1.0;
  double knot_space_ = 0.02;
  double time_padding_ = 0.015;

  //filter uv points which lie out of the border. 
  int border_filter_uv_ = 10; 
  bool optimize_time_offset_ = false;
  bool lock_traj_lidar_in_2nd_stage_ = false;
  bool lock_traj_lidar_in_3rd_stage_ = false;

  bool simulation_ = false;
  bool save_rendered_map_ = false;
  std::string pcd_outpath_;
  std::string calib_result_outpath_;
  std::string bag_name_;

  ros::Publisher map_cloud_publisher_;
};


// Fill in landmark_db_, views_db_ and integration_frames_cam_.
void LIinitializer::LoadOrbResults(const std::string& orb_res_path){
  std::ifstream ifs(orb_res_path);
  if(!ifs.is_open()){
    LOG(ERROR) << "Open ORB result file failed!";
    return;
  }
  std::string line;
  std::vector<std::string> tokens;
  int64_t last_frame_id = 0;
  double stamp_min = std::numeric_limits<double>::max();
  double stamp_max = std::numeric_limits<double>::min();
  while(std::getline(ifs, line)){
    tokens.clear();
    tokens = SplitString(line, ' ');
    if(tokens.empty()) break;
    if (tokens[0] == "FramePose"){
      CHECK(tokens.size() == 9)<<"Wrong line!";
      IntegrationFrame frame;
      std::istringstream iss(tokens[1]);
      iss >> frame.timestamp;
      last_frame_id = frame.timestamp;//按Frame顺序组织的
      stamp_min = std::min(stamp_min, double(last_frame_id) * 1e-9);
      stamp_max = std::max(stamp_max, double(last_frame_id) * 1e-9);
      Eigen::Vector3d t = Eigen::Vector3d(atof(tokens[2].c_str()), 
               atof(tokens[3].c_str()), atof(tokens[4].c_str()));
      Eigen::Quaterniond q = Eigen::Quaterniond(
        atof(tokens[8].c_str()), atof(tokens[5].c_str()), 
        atof(tokens[6].c_str()), atof(tokens[7].c_str())); 
      frame.Tcw.block<3,1>(0, 3) = t;
      frame.Tcw.block<3,3>(0, 0) = q.toRotationMatrix();
      integration_frames_cam_.push_back(frame);
    }else if(tokens[0] == "UV"){
      //UV是按照Keyframe组织的
      CHECK((tokens.size() - 2) % 3 == 0)<<"Wrong line!";
      int num = (tokens.size() - 2) / 3;
      std::map<int64_t, Eigen::Vector2d> obs_pair;
      int64_t frameid;
      std::istringstream iss(tokens[1]);
      iss >> frameid;
      for(int i = 0; i < num; ++i){
        int idx  = 2 + i * 3;//0:UV 1:frame_id
        Eigen::Vector2d uv = Eigen::Vector2d(
          atof(tokens[idx].c_str()), atof(tokens[idx+1].c_str()));
        int64_t mp_id;
        std::istringstream iss(tokens[idx+2]);
        iss >> mp_id;
        obs_pair[mp_id] = uv;
      }
      uv_points[frameid] = obs_pair;
      std::shared_ptr<kontiki::sfm::View> view = std::make_shared<
        kontiki::sfm::View>(frameid, frameid*1e-9);
      views_db_[frameid] = view;
    }else if(tokens[0] == "MapPoint"){
      CHECK(tokens.size() == 6)<<"Wrong line!";
      int64_t lm_id, ref_id;
      std::istringstream iss_lm(tokens[1]);
      iss_lm >> lm_id;
      std::istringstream iss_ref(tokens[5]);
      iss_ref >> ref_id;

      //pos in reference frame
      Eigen::Vector3d pos = Eigen::Vector3d(atof(tokens[2].c_str()), 
          atof(tokens[3].c_str()), atof(tokens[4].c_str()));
      
      auto view_ref = views_db_.find(ref_id);
      if(view_ref == views_db_.end()){
        LOG(INFO)<<"Wrong ID, no reference frame found!"<<ref_id;
        continue;
      }
      
      if(uv_points.find(ref_id) == uv_points.end()) continue;
      auto it_uv = uv_points[ref_id].find(lm_id);
      if(it_uv == uv_points[ref_id].end()) continue;

      Eigen::Vector2d euv = it_uv->second;
      int border = border_filter_uv_;
      if(euv[0] < border || euv[1] < border 
          || euv[0] > camera_params_.col-border
          || euv[1] > camera_params_.row-border){
        continue;
      }//剔除边缘点

      auto iter = landmark_db_.find(lm_id);
      if(iter == landmark_db_.end()){
        std::shared_ptr<kontiki::sfm::Landmark> lm = 
          std::make_shared<kontiki::sfm::Landmark>();
        
        //TODO: Update this after VI initialization
        lm->set_inverse_depth(1. / (pos[2]+1e-15));
        
        auto ref_obs_ptr = view_ref->second->CreateObservation(lm, euv);
        lm->set_reference(ref_obs_ptr);
        // lm->Lock(true); 
        landmark_db_.insert({lm_id, lm});

        for(auto it = views_db_.begin(); it != views_db_.end(); it++){
          if(it == view_ref) continue;
          auto it_obs = uv_points[it->first].find(lm_id);
          if(it_obs != uv_points[it->first].end()){
            Eigen::Vector2d euv_obs = it_obs->second;
            auto obs_ptr = it->second->CreateObservation(lm, euv_obs);
          }
        }  
      }
    }
  }
  ifs.close();
  
  //update start and end time
  // start_time_ = stamp_min;
  // end_time_ = stamp_max;
  LOG(INFO)<<"Load ORB-SLAM result succeeded!";
  LOG(INFO)<<"View size: "<<views_db_.size();
  LOG(INFO)<<"Landmark size: "<<landmark_db_.size();
  LOG(INFO)<<"UV points size: "<<uv_points.size();
  // for(auto it = landmark_db_.begin(); it != landmark_db_.end(); ++it){
  //   LOG(INFO)<<it->second->observations().size();
  // }
}


void LIinitializer::ReadPoseGT(const std::string& pose_file_path){
  std::ifstream ifs(pose_file_path);
  if(!ifs.is_open()){
    LOG(ERROR)<<"Open pose file failed! "<<pose_file_path;
  }
  std::string line;

  Eigen::Vector3d last_pos;
  Eigen::Quaterniond last_ori;
  
  double stamp_min = std::numeric_limits<double>::max();
  double stamp_max = std::numeric_limits<double>::min();
  while(std::getline(ifs, line)){
    std::vector<std::string> words = SplitString(line, ' ');
    if(words.size() != 8) break;
    std::istringstream iss(words[0]);
    int64_t stamp;
    iss >> stamp;
    stamp_min = std::min(stamp_min, double(stamp) * 1e-9);
    stamp_max = std::max(stamp_max, double(stamp) * 1e-9);

    auto pos = Eigen::Vector3d(atof(words[1].c_str()), 
               atof(words[2].c_str()), atof(words[3].c_str()));
    auto rot = Eigen::Quaterniond(
      atof(words[4].c_str()), atof(words[5].c_str()), 
      atof(words[6].c_str()), atof(words[7].c_str())); 
    
    // tsf.pos = R_zxy_to_xyz * pos;
    // tsf.rot = Eigen::Quaterniond(R_zxy_to_xyz * rot.toRotationMatrix());
    // tsf.rot = Q_zxy_to_xyz * rot * Q_zxy_to_xyz.inverse();
    // tsf.pos = pos;
    // tsf.rot = rot;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3,1>(0, 3) = pos;
    pose.block<3,3>(0, 0) = rot.toRotationMatrix();
    loam_poses_.push_back({stamp*1e-9, pose});
    loam_poses_map_[stamp] = pose;

    if(integration_frames_lidar_.size() > 0){
      if(last_ori.angularDistance(rot)*180. / M_PI < 5. 
        && (last_pos - pos).norm() < 0.1){
        continue;
      }
    }
    // fill in frames
    IntegrationFrame frame;
    frame.timestamp = stamp;
    frame.Tcw = pose;
    integration_frames_lidar_.push_back(frame);
    last_pos = pos;
    last_ori = rot;
  }
  ifs.close();
  // start_time_ = stamp_min;
  // end_time_ = stamp_max;
  
  LOG(INFO)<<"Load key poses " <<integration_frames_lidar_.size();
  LOG(INFO)<<"Load all poses " <<loam_poses_.size();
}


void LIinitializer::CIoptimize(){
  if(!init_cam_ok_) {
    LOG(ERROR)<<"The Camera should be inited first!";
    // return;
    T_I_C_ << -0.22, 0.02, 0.22;
    R_I_C_ << Eigen::Matrix3d::Identity();
  }
  
  int nouse;
  SetCalibWeights(nouse); 
  traj_manager_->getCalibParamManager()->set_p_CinI(T_I_C_);
  traj_manager_->getCalibParamManager()->set_q_CtoI(Eigen::Quaterniond(R_I_C_));
  
  for(const auto& imu: imu_quene_for_traj_){
    traj_manager_->feedIMUData(imu);
  }
  traj_manager_->initialSO3TrajWithGyro();
  traj_manager_->trajInitFromVisualFrames(views_db_, optimize_time_offset_);
}

void LIinitializer::LCIoptimize(){
  if(!init_cam_ok_ || !init_lidar_ok_) {
    LOG(ERROR)<<"The Camera and LiDAR should be inited first!";
    return;
    
    //for non-initialization test
    std::default_random_engine e_xyz;
    std::default_random_engine e_rpy;
    std::uniform_real_distribution<double>xyz(-2., 2.);
    std::uniform_real_distribution<double>rpy(-0.5, 0.5);
    
    T_I_C_ << xyz(e_xyz), xyz(e_xyz), xyz(e_xyz);
    R_I_C_ = yprToQuaternion(rpy(e_rpy),rpy(e_rpy),rpy(e_rpy));
    T_I_L_ << xyz(e_xyz), xyz(e_xyz), xyz(e_xyz);
    R_I_L_ =  yprToQuaternion(rpy(e_rpy),rpy(e_rpy),rpy(e_rpy));
  }
  
  int nouse;
  SetCalibWeights(nouse); 
  traj_manager_->getCalibParamManager()->set_p_LinI(T_I_L_);
  traj_manager_->getCalibParamManager()->set_p_CinI(T_I_C_);
  traj_manager_->getCalibParamManager()->set_q_LtoI(Eigen::Quaterniond(R_I_L_));
  traj_manager_->getCalibParamManager()->set_q_CtoI(Eigen::Quaterniond(R_I_C_));
  
  //save initial estimates
  cv::FileStorage fs(
    calib_result_outpath_+"/"+bag_name_+".yaml",cv::FileStorage::WRITE);
  Eigen::Matrix4d T_I2C_init = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_I2L_init = Eigen::Matrix4d::Identity();
  T_I2C_init.block<3,3>(0, 0) = R_I_C_.transpose();
  T_I2C_init.block<3,1>(0, 3) = R_I_C_.transpose()*(-T_I_C_);
  T_I2L_init.block<3,3>(0, 0) = R_I_L_.transpose();
  T_I2L_init.block<3,1>(0, 3) = R_I_L_.transpose()*(-T_I_L_);
  
  cv::Mat T_I2C_init_cv, T_I2L_init_cv;
  cv::eigen2cv(T_I2C_init, T_I2C_init_cv);
  cv::eigen2cv(T_I2L_init, T_I2L_init_cv);
  fs<<"Initial_T_cam_imu"<<T_I2C_init_cv;  
  fs<<"Initial_T_lidar_imu"<<T_I2L_init_cv;

  for(const auto& imu: imu_quene_for_traj_){
    traj_manager_->feedIMUData(imu);
  }
  traj_manager_->initialSO3TrajWithGyro();
  
  // traj_manager_->trajInitFromVisualFrames(views_db_, optimize_time_offset_);
  // LOG(INFO)<<traj_manager_->getCalibParamManager()->q_CtoI.toRotationMatrix();
  // return;


  calib_step_ = InitializationDone;
  DataAssociation();
  if (calib_step_ != DataAssociationDone) {
    ROS_WARN("[BatchOptimization] Need status: DataAssociationDone.");
    return;
  }
  traj_manager_->trajInitFromSurfel(surfel_association_, optimize_time_offset_);
  calib_step_ = BatchOptimizationDone;
  
  for(int i = 0; i<2; ++i){
    Refinement();
  }

  //save lidar-imu result without visual observations. 1st stage
  Eigen::Matrix4d T_I2L = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R_L2I = 
    traj_manager_->getCalibParamManager()->q_LtoI.toRotationMatrix();
  Eigen::Vector3d t_LinI = traj_manager_->getCalibParamManager()->p_LinI;
  Eigen::Vector3d t_IinL = R_L2I.transpose()*(-t_LinI);
  T_I2L.block<3,3>(0,0) = R_L2I.transpose();
  T_I2L.block<3,1>(0,3) = t_IinL;
  
  cv::Mat T_I2L_cv; 
  cv::eigen2cv(T_I2L, T_I2L_cv);
  fs<<"T_lidar_imu_1st"<<T_I2L_cv;

  // // 先分别解算LI和VI，获取LV初始值，以及3D视觉点，然后用ICP将3D视觉点与地图配准.
  // // 进一步筛选3D点到surfel的关联，最后所有约束统一再优化一遍
  traj_manager_->trajInitFromLVIdata(views_db_, surfel_association_, 
    optimize_time_offset_, lock_traj_lidar_in_2nd_stage_);
  
  //save to file, 2nd stage
  Eigen::Matrix4d T_I2C = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R_C2I = 
    traj_manager_->getCalibParamManager()->q_CtoI.toRotationMatrix();
  Eigen::Vector3d t_CinI = traj_manager_->getCalibParamManager()->p_CinI;
  Eigen::Vector3d t_IinC = R_C2I.transpose()*(-t_CinI);
  T_I2C.block<3,3>(0,0) = R_C2I.transpose();
  T_I2C.block<3,1>(0,3) = t_IinC;
  
  cv::Mat T_I2C_cv; 
  cv::eigen2cv(T_I2C, T_I2C_cv);
  fs<<"T_cam_imu_2nd"<<T_I2C_cv;

  //save lidar-imu result with visual observations.
  T_I2L = Eigen::Matrix4d::Identity();
  R_L2I = 
    traj_manager_->getCalibParamManager()->q_LtoI.toRotationMatrix();
  t_LinI = traj_manager_->getCalibParamManager()->p_LinI;
  t_IinL = R_L2I.transpose()*(-t_LinI);
  T_I2L.block<3,3>(0,0) = R_L2I.transpose();
  T_I2L.block<3,1>(0,3) = t_IinL;
  
  cv::eigen2cv(T_I2L, T_I2L_cv);
  fs<<"T_lidar_imu_2nd"<<T_I2L_cv;
  
  //associate visual points to surfels
  Eigen::Quaterniond q_LtoC;
  Eigen::Vector3d t_LinC;
  const auto para_server = traj_manager_->getCalibParamManager();
  q_LtoC =  para_server->q_CtoI.conjugate() * para_server->q_LtoI;
  t_LinC =  para_server->q_CtoI.conjugate() * (
    para_server->p_LinI - para_server->p_CinI);
  std::map<kontiki::sfm::Landmark*, size_t> lm_splane;
  surfel_association_->associateVisualPointsWithPlanes(
    traj_manager_, q_LtoC, t_LinC, landmark_db_, lm_splane);
  
  traj_manager_->trajInitFromLVIdata(
    views_db_, surfel_association_, lm_splane,
    optimize_time_offset_, lock_traj_lidar_in_3rd_stage_);

  // q_LtoC =  para_server->q_CtoI.conjugate() * para_server->q_LtoI;
  // t_LinC =  para_server->q_CtoI.conjugate() * (
  //   para_server->p_LinI - para_server->p_CinI);
  // lm_splane.clear();
  // surfel_association_->associateVisualPointsWithPlanes(
  //   traj_manager_, q_LtoC, t_LinC, landmark_db_, lm_splane);  

  // PublishTrajectory();
  // SaveOptimizedPCDMap(pcd_outpath_);
  // ReprojectPointCloudToImage();
  if(save_rendered_map_){
    RenderMap();
  }
  

  //save to file, 3rd stage
  T_I2C = Eigen::Matrix4d::Identity();
  R_C2I = traj_manager_->getCalibParamManager()->q_CtoI.toRotationMatrix();
  t_CinI = traj_manager_->getCalibParamManager()->p_CinI;
  t_IinC = R_C2I.transpose()*(-t_CinI);
  T_I2C.block<3,3>(0,0) = R_C2I.transpose();
  T_I2C.block<3,1>(0,3) = t_IinC;
  
  T_I2C_cv; 
  cv::eigen2cv(T_I2C, T_I2C_cv);
  fs<<"T_cam_imu_3rd"<<T_I2C_cv;

  T_I2L = Eigen::Matrix4d::Identity();
  R_L2I = traj_manager_->getCalibParamManager()->q_LtoI.toRotationMatrix();
  t_LinI = traj_manager_->getCalibParamManager()->p_LinI;
  t_IinL = R_L2I.transpose()*(-t_LinI);
  T_I2L.block<3,3>(0,0) = R_L2I.transpose();
  T_I2L.block<3,1>(0,3) = t_IinL;
  
  cv::eigen2cv(T_I2L, T_I2L_cv);
  fs<<"T_lidar_imu_3rd"<<T_I2L_cv;
  
  cv::Mat err_uv, err_acc, err_gyr, err_surfel;
  cv::eigen2cv(traj_manager_->err_cam_, err_uv);
  cv::eigen2cv(traj_manager_->err_acc_, err_acc);
  cv::eigen2cv(traj_manager_->err_gyr_, err_gyr);
  cv::eigen2cv(traj_manager_->err_surfel_, err_surfel);
  fs<<"Error_uv"<<err_uv;
  fs<<"Error_acc"<<err_acc;
  fs<<"Error_gyr"<<err_gyr;
  fs<<"Error_surfel"<<err_surfel;

  fs.release();
}


void LIinitializer::RenderMap(){
  scan_undistortion_->undistortScanInMap();
  pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
  pcl::PointXYZRGB pt_rgb; 
  colored_cloud.resize(scan_undistortion_->get_map_cloud()->size());

  Eigen::Quaterniond q_L0_to_G;
  Eigen::Vector3d p_L0_in_G;
  double map_start_time = traj_manager_->get_map_time();
  if(!traj_manager_->evaluateLidarPose(map_start_time, q_L0_to_G, p_L0_in_G)){
    LOG(WARNING)<<"Error evaluate lidar pose!";
    return;
  }
  q_L0_to_G.normalize();
  Eigen::Matrix4d T_L0inG = Eigen::Matrix4d::Identity();
  T_L0inG.block<3,3>(0,0) = q_L0_to_G.toRotationMatrix();
  T_L0inG.block<3,1>(0,3) = p_L0_in_G;

  int step = dataset_reader_->get_image_data().size() / 10;
  const auto camera = traj_manager_->getCameraModel();
  
  std::vector<std::vector<double>> color_points_mashlab={};
  for(int i = 50; i < dataset_reader_->get_image_data().size(); i+=step){
    const cv::Mat& img = dataset_reader_->get_image_data()[i].image;
    double img_t = dataset_reader_->get_image_data()[i].timestamp;
    Eigen::Quaterniond q_CtoG;
    Eigen::Vector3d p_CinG;
    if(!traj_manager_->evaluateCameraPose(img_t, q_CtoG, p_CinG)){
      LOG(WARNING)<<"Error evaluate camera pose at "<<img_t;
      continue;
    }
    Eigen::Matrix4d T_CinG= Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_GtoC= Eigen::Matrix4d::Identity();
    q_CtoG.normalize();
    T_CinG.block<3,3>(0,0) = q_CtoG.toRotationMatrix();
    T_CinG.block<3,1>(0,3) = p_CinG;
    T_GtoC = T_CinG.inverse();

    Eigen::Vector4d ept, pt_G, pt_C;
    for(int j = 0; j < scan_undistortion_->get_map_cloud()->points.size(); ++j){
      const auto& pt = scan_undistortion_->get_map_cloud()->points[j];
      if(pcl_isnan(pt.x) || pcl_isnan(pt.y) || pcl_isnan(pt.z)) continue;
      ept << pt.x, pt.y, pt.z, 1;
      pt_G = T_L0inG * ept;
      pt_C = T_GtoC * pt_G;
      
      if(pt_C[2] < 0.1 || pt_C[2] > 15) continue;
      
      pt_rgb.x = pt.x;
      pt_rgb.y = pt.y;
      pt_rgb.z = pt.z;
      colored_cloud.points[j] = pt_rgb;

      // reproject
      // Eigen::Vector2d uv = camera->Project(pt_C);
      Eigen::Vector3d pt_C_norm;
      Eigen::Vector2d uv;
      pt_C_norm << pt_C[0]/pt_C[2], pt_C[1]/pt_C[2], 1;
      double tmpx = pt_C_norm[0];
      double tmpy = pt_C_norm[1];
      double r2 = pt_C_norm[0]*pt_C_norm[0] + pt_C_norm[1]*pt_C_norm[1];
      double tmpdist = 1 + camera_params_.k1*r2 + camera_params_.k2*r2*r2 
              + camera_params_.k3*r2*r2*r2;
      uv[0] = tmpx * tmpdist + 2 *  camera_params_.p1 * tmpx * tmpy 
              +  camera_params_.p2 * (r2 + 2 * tmpx * tmpx);
      uv[1] = tmpy * tmpdist + camera_params_.p1 * (r2 + 2 * tmpy * tmpy) 
              + 2 * camera_params_.p2 * tmpx * tmpy;
      uv[0] = camera_params_.fx * uv[0] + camera_params_.cx;
      uv[1] = camera_params_.fy * uv[1] + camera_params_.cy;

      if(int(uv[0]) < 0 || int(uv[1]) < 0 || int(uv[0]) > (img.cols - 1) 
          || int(uv[1]) > (img.rows - 1)){
        continue;
      }
      u_char bgr = img.at<u_char>(int(uv[1]), int(uv[0]));
      pt_rgb.r = bgr;
      pt_rgb.g = bgr;
      pt_rgb.b = bgr;
      // cv::Vec3b bgr = img.at<cv::Vec3b>(int(uv[1]), int(uv[0]));
      // pt_rgb.r = bgr[2];
      // pt_rgb.g = bgr[1];
      // pt_rgb.b = bgr[0];
      colored_cloud.points[j] = pt_rgb;

      std::vector<double> ptc;
      ptc = {pt_C[0], pt_C[1], pt_C[2], 
        double(pt_rgb.r),  double(pt_rgb.g),  double(pt_rgb.b)};
      color_points_mashlab.push_back(ptc);
    }
    break;
  }


  pcl::io::savePCDFileASCII(pcd_outpath_, colored_cloud);
  // std::ofstream ofs(pcd_outpath_);
  // if(!ofs.is_open()) return;
  // for(const auto& cp : color_points_mashlab){
  //   ofs<<cp[0]<<","<<cp[1]<<","<<cp[2]<<","<<cp[3]<<","<<cp[4]<<","<<cp[5]<<"\n";
  // }
  // ofs.close();
}

// void LIinitializer::PublishMapCloud(){
//   scan_undistortion_->undistortScanInMap();
//   if(map_cloud_publisher_.getNumSubscribers() != 0){
//     sensor_msgs::PointCloud2 mp;
//     mp.header.stamp = ros::Time::now();
//     mp.header.frame_id = "map";
//     mp.data.resize(scan_undistortion_->get_map_cloud()->size());
//     for(int i = 0; i<scan_undistortion_->get_map_cloud()->size(); ++i){
//       const auto& p = scan_undistortion_->get_map_cloud()->points[i];
//       mp.data[i].x = p.x;
//       mp.data[i].y = p.y;
//       mp.data[i].z = p.z;
//     }
//     ros::Rate r(0.5);
//     while(ros::ok()){
//       map_cloud_publisher_.publish(mp);
//       r.sleep();
//     }
//   }
// }

void LIinitializer::PublishTrajectory(){
  nav_msgs::Path traj_lidar;
  nav_msgs::Path traj_opt;
  nav_msgs::Path traj_cam;

  geometry_msgs::PoseStamped stp_pose;
  geometry_msgs::Pose pose;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "map";
  stp_pose.header = header;
  traj_lidar.header = header;
  traj_opt.header = header;

  //loam result
  double lmapping_time = integration_frames_lidar_.front().timestamp * 1e-9;
  Eigen::Quaterniond qwl;
  Eigen::Vector3d pwl;
  Eigen::Matrix4d Twl = Eigen::Matrix4d::Identity();
  traj_manager_->evaluateLidarPose(lmapping_time, qwl, pwl);
  Twl.block<3,1>(0,3) = pwl;
  Twl.block<3,3>(0,0) = qwl.toRotationMatrix();
  for(const auto& frame: integration_frames_lidar_){
    Eigen::Matrix4d Tl = frame.Tcw;
    Eigen::Matrix4d TG = Twl*Tl;
    pose.position.x = TG.block<3,1>(0,3)[0];
    pose.position.y = TG.block<3,1>(0,3)[1];
    pose.position.z = TG.block<3,1>(0,3)[2];
    Eigen::Quaterniond q = Eigen::Quaterniond(TG.block<3,3>(0,0));
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    stp_pose.pose = pose;
    traj_lidar.poses.push_back(stp_pose);
  }
  
  //optimized result
  double start_time = imu_quene_for_traj_.front().timestamp;
  double end_time = imu_quene_for_traj_.back().timestamp;
  double t = start_time;
  while(t < end_time){
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    traj_manager_->evaluateLidarPose(t, q, p);
    pose.position.x = p[0];
    pose.position.y = p[1];
    pose.position.z = p[2];
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    stp_pose.pose = pose;
    traj_opt.poses.push_back(stp_pose);
    t += 0.05;
  }
  
  ros::NodeHandle nh("~");
  ros::Rate r(10);
  ros::Publisher pub_loam = nh.advertise<nav_msgs::Path>("path_loam", 200);
  ros::Publisher pub_opt = nh.advertise<nav_msgs::Path>("path_opt", 200);
  while(ros::ok()){
    pub_loam.publish(traj_lidar);
    pub_opt.publish(traj_opt);
    r.sleep();
  }
}

void LIinitializer::SetCalibWeights(int& skip_frame){
  if(!traj_manager_) return;
  ros::NodeHandle nh("~");
  double w_acc, w_gyr, w_pos, w_rot, w_cam, w_ls, w_vs;
  nh.getParam("w_acc", w_acc);
  nh.getParam("w_gyr", w_gyr);
  nh.getParam("w_pos", w_pos);
  nh.getParam("w_rot", w_rot);
  nh.getParam("w_cam", w_cam);
  nh.getParam("w_lidar_surfel", w_ls);
  nh.getParam("w_visual_surfel", w_vs);
  nh.getParam("skip_frame", skip_frame);
  traj_manager_->getCalibParamManager()->global_opt_acce_weight = w_acc;
  traj_manager_->getCalibParamManager()->global_opt_gyro_weight = w_gyr;
  traj_manager_->getCalibParamManager()->global_opt_lidar_weight = w_ls;
  traj_manager_->getCalibParamManager()->global_opt_visual_surfel_weight = w_vs;
  traj_manager_->getCalibParamManager()->global_opt_cam_weight = w_cam;
  traj_manager_->getCalibParamManager()->global_opt_pos_weight = w_pos;
  traj_manager_->getCalibParamManager()->global_opt_rot_weight = w_rot;

  LOG(INFO)<<"Set "<<"w_acc: "<<w_acc<<", w_gyr: "<<w_gyr<<", w_lidar_surfel: "
           <<w_ls<<", w_camera: "<<w_cam<<", w_pos: "
           <<w_pos<<", w_rot: "<<w_rot;
  LOG(INFO)<<"Set skip frame: "<<skip_frame;
}

void LIinitializer::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg){
  imu_recieved_ = true;
  if(imu_msg->header.stamp.toSec() < start_time_ 
      || imu_msg->header.stamp.toSec() > end_time_ ){
    return;
  }
  imu_cache_cam_.push_back(*imu_msg);
  imu_cache_lidar_.push_back(*imu_msg);
  
  licalib::IO::IMUData imu_for_traj;
  imu_for_traj.timestamp = imu_msg->header.stamp.toSec();
  Eigen::Vector3d acc, gyr;
  acc << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z;
  gyr << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z;
  imu_for_traj.gyro = gyr;
  imu_for_traj.accel = acc;

  imu_quene_for_traj_.push_back(imu_for_traj);
}

void LIinitializer::PopOldIMU(
    const double stamp, std::deque<sensor_msgs::Imu>& imu_cache){
  while(1){
    if(imu_cache.empty()) break;
    if(imu_cache.front().header.stamp.toSec() < stamp){
      imu_cache.pop_front();
    }else{
      break;
    }
  }
}


void LIinitializer::RemoveOverTimeFrames(
    const std::deque<sensor_msgs::Imu>& imu_cache,
    std::deque<IntegrationFrame>& integration_frames){
  while(1){
    if(integration_frames.empty()) break;
    if(integration_frames.front().timestamp * 1e-9 
        < imu_cache.front().header.stamp.toSec()){
      integration_frames.pop_front();
    }else{
      break;
    }
  }
  while(1){
    if(integration_frames.empty()) break;
    if(integration_frames.back().timestamp*1e-9 
      > imu_cache.back().header.stamp.toSec()){
      integration_frames.pop_back();
    }else{
      break;
    }
  }
}


void LIinitializer::ComputeIntegrationForFrames(
    std::deque<IntegrationFrame>& integration_frames,
    std::deque<sensor_msgs::Imu>& imu_cache){
  
  RemoveOverTimeFrames(imu_cache, integration_frames);
  size_t size = integration_frames.size();  
  Eigen::Vector3d zero_vec;
  zero_vec << 0, 0, 0;
  double last_imu_time = -1.;

  for(size_t i = 1; i < size; ++i){
    auto& intframe = integration_frames[i];
    PopOldIMU(integration_frames[i-1].timestamp * 1e-9, imu_cache);
    std::shared_ptr<IntegrationBase> integrator = 
      std::make_shared<IntegrationBase>(zero_vec, zero_vec, imu_noise_);

    //integrate
    if(!imu_cache.empty()){
      double cur_stamp = intframe.timestamp * 1e-9;
      size_t imu_num = 0;
      while(1){
        if(imu_cache.empty()) break;
        if(imu_cache.front().header.stamp.toSec() < cur_stamp ){
          imu_num++;
          auto imu_msg = imu_cache.front();
          double cur_imu_time = imu_msg.header.stamp.toSec();
          Eigen::Vector3d acc, gyr;
          acc << imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z;
          gyr << imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z;
          if(last_imu_time < 0){
            integrator->push_back(0.001, acc, gyr);//TODO: Read IMU Frequency
          }else{
            integrator->push_back(cur_imu_time - last_imu_time, acc, gyr);
          }
          last_imu_time = cur_imu_time;
          imu_cache.pop_front();
        }else{
          if(imu_num < 10) {
            LOG(ERROR)<<"No imu data, somthing wrong. Skip this frame. "<<imu_num;
            // return;
          }
          break;
        }
      }
    }else{
      LOG(ERROR)<<"No imu data, somthing wrong. Pass this frame.";
      return;
    }
    intframe.integrator = integrator;
  }
  LOG(INFO)<<"Compute integration finished!";
}

void LIinitializer::EstimateInitExtrinsicCI(){
  for(int i = 0; i < integration_frames_cam_.size() - 1; ++i){
    Eigen::Matrix3d relative_rot = 
      integration_frames_cam_[i].Tcw.block<3, 3>(0, 0).transpose() 
      * integration_frames_cam_[i+1].Tcw.block<3, 3>(0, 0);
    init_rot_est_ok_cam_ = rot_estimator_.CalibrationExRotationLiDAR(
      relative_rot, integration_frames_cam_[i+1].integrator->delta_q, R_I_C_);
    if(init_rot_est_ok_cam_){
      LOG(INFO)<<"Estimated initial R_I_C: "<<R_I_C_;
      LOG(INFO)<<"Estimated initial Euler R_ItoC: "<<R_I_C_.transpose().eulerAngles(0,1,2).transpose()*180./M_PI;
      break;
    }
  }
  if(!init_rot_est_ok_cam_){
    LOG(ERROR)<<"Failed init R_I_C!";
    rot_estimator_.Reset();
    return;
  }
  
  // std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> nouse;
  const int winsize = 10;//TODO
  Eigen::Vector3d Bgs, g, T_I_C;
  Eigen::VectorXd x;
  for(int i = winsize; i < integration_frames_cam_.size(); ++i){
    Eigen::Matrix4d T_inv = integration_frames_cam_[i - winsize].Tcw.inverse();
    std::deque<ImageFrame> win_frames={};
    for(int k = i - winsize; k < i; ++k){
      Eigen::Matrix4d Tk = T_inv * integration_frames_cam_[k].Tcw;
      ImageFrame kf;
      kf.R = Tk.block<3,3>(0,0) * R_I_C_.transpose();//注意，这里需要转换到IMU
      kf.T = Tk.block<3,1>(0,3);
      kf.pre_integration = integration_frames_cam_[k].integrator.get();
      win_frames.push_back(kf);
    }
     
    init_cam_ok_ = VisualIMUAlignment(win_frames, &Bgs, g, T_I_C_, x, false);
    if(init_cam_ok_){
      LOG(INFO)<<"Estimated initial T_I_C: "<<T_I_C_.transpose();
      
      // 此处不进行逆深度更新似乎收敛更快
      // Assign inverse depth for each landmark
      double scale = (x.tail<1>())(0);
      for(auto it = landmark_db_.begin(); it !=landmark_db_.end(); ++it){
        double inv_depth = it->second->inverse_depth();
        it->second->set_inverse_depth(inv_depth / ( scale+1e-15));
      }
      
      // Copy gravity and mark the associated time
      g_I_t_cam_.segment<3>(0) = R_I_C_ * g;
      g_I_t_cam_[3] = integration_frames_cam_[i - winsize].timestamp * 1e-9;
      rot_estimator_.Reset();
      return;
    }
  }

  LOG(ERROR)<<"Failed init T_I_C!";
  rot_estimator_.Reset();
}

void LIinitializer::EstimateInitExtrinsicLI(){
  const int winsize = 10;//TODO
  for(int i = 0; i < integration_frames_lidar_.size() - 1; ++i){
    Eigen::Matrix3d relative_rot = integration_frames_lidar_[
      i].Tcw.block<3, 3>(0, 0).transpose() * integration_frames_lidar_[
        i+1].Tcw.block<3, 3>(0, 0);
    init_rot_est_ok_lidar_ = rot_estimator_.CalibrationExRotationLiDAR(
      relative_rot, integration_frames_lidar_[
        i+1].integrator->delta_q, R_I_L_);
    if(init_rot_est_ok_lidar_){
      break;
    }
  }
  if(!init_rot_est_ok_lidar_){
    rot_estimator_.Reset();
    LOG(ERROR)<<"Failed init R_I_L!";
    return;
  }
  
  Eigen::Vector3d Bgs, g;
  Eigen::VectorXd x;
  for(int i = winsize; i < integration_frames_lidar_.size(); ++i){
    Eigen::Matrix4d T_inv = integration_frames_lidar_[i - winsize].Tcw.inverse();
    std::deque<ImageFrame> win_frames={};
    for(int k = i - winsize; k < i; ++k){
      Eigen::Matrix4d Tk = T_inv * integration_frames_lidar_[k].Tcw;
      ImageFrame kf;
      kf.R = Tk.block<3,3>(0,0) * R_I_L_.transpose();//注意，这里需要转换到IMU
      kf.T = Tk.block<3,1>(0,3);
      kf.pre_integration = integration_frames_lidar_[k].integrator.get();
      win_frames.push_back(kf);
    }
     
    init_lidar_ok_ = VisualIMUAlignment(win_frames, &Bgs, g, T_I_L_, x, true);
    if(init_lidar_ok_){
      LOG(INFO)<<"Estimated initial T_I_L: "<<T_I_L_.transpose();
      g_I_t_lidar_.segment<3>(0) = R_I_L_ * g;
      g_I_t_lidar_[3] = integration_frames_lidar_[i - winsize].timestamp * 1e-9;
      calib_step_ = InitializationDone;
      rot_estimator_.Reset();
      return;
    }
  }
  rot_estimator_.Reset();
  LOG(ERROR)<<"Failed init T_I_L!";
}

void LIinitializer::NdtMapping(){
  for(const TPointCloud& raw_scan: dataset_reader_->get_scan_data()) {
    VPointCloud::Ptr cloud(new VPointCloud);
    TPointCloud2VPointCloud(raw_scan.makeShared(), cloud);
    double scan_timestamp = pcl_conversions::fromPCL(
      raw_scan.header.stamp).toSec();
    lidar_odom_->feedScan(scan_timestamp, cloud);
  }
  const auto& odom_data = lidar_odom_->get_odom_data();
  const auto& key_frame_idx = lidar_odom_->getKeyFrameIndex();
  for(int id: key_frame_idx){
    IntegrationFrame frame;
    frame.timestamp = int64_t(odom_data[id].timestamp*1e9);
    frame.Tcw = odom_data[id].pose;
    integration_frames_lidar_.push_back(frame);
  }
}


void LIinitializer::DataAssociation() {
  std::cout << "[Association] start ...." << std::endl;
  TicToc timer;
  timer.tic();

  /// set surfel pap
  if (InitializationDone == calib_step_ ) {
    Mapping();
    scan_undistortion_->undistortScanInMap(lidar_odom_->get_odom_data_map());
    surfel_association_->setSurfelMap(lidar_odom_->getNDTPtr(), map_time_);
  } else if (BatchOptimizationDone == calib_step_ || RefineDone == calib_step_) {
    scan_undistortion_->undistortScanInMap();

    plane_lambda_ = 0.7;
    surfel_association_->setPlaneLambda(plane_lambda_);
    auto ndt_omp = LiDAROdometry::ndtInit(ndt_resolution_);
    ndt_omp->setInputTarget(scan_undistortion_->get_map_cloud());
    surfel_association_->setSurfelMap(ndt_omp, map_time_);
  } else {
      ROS_WARN("[DataAssociation] Please follow the step.");
      return;
  }

  /// get association
  for (auto const &scan_raw : dataset_reader_->get_scan_data()) {
    auto iter = scan_undistortion_->get_scan_data_in_map().find(
            scan_raw.header.stamp);
    if (iter == scan_undistortion_->get_scan_data_in_map().end()) {
      continue;
    }
    surfel_association_->getAssociation(iter->second, scan_raw.makeShared(), 2);
  }
  surfel_association_->averageTimeDownSmaple();
  std::cout << "Surfel point number: "
            << surfel_association_->get_surfel_points().size() << std::endl;

  if (surfel_association_->get_surfel_points().size() > 10){
    calib_step_ = DataAssociationDone;
  } else {
    ROS_WARN("[DataAssociation] fails.");
  }
}

void LIinitializer::BatchOptimization() {
  if (DataAssociationDone != calib_step_) {
    ROS_WARN("[BatchOptimization] Need status: DataAssociationDone.");
    return;
  }
  std::cout << "\n================ Iteration " << iteration_step_ << " ==================\n";

  TicToc timer;
  timer.tic();
  traj_manager_->trajInitFromSurfel(surfel_association_, opt_time_offset_);

  calib_step_ = BatchOptimizationDone;
  saveCalibResult(cache_path_ + "/calib_result.csv");
}

void LIinitializer::Refinement() {
  if (BatchOptimizationDone > calib_step_) {
    ROS_WARN("[Refinement] Need status: BatchOptimizationDone.");
    return;
  }
  iteration_step_++;
  std::cout << "\n================ Iteration " << iteration_step_ << " ==================\n";

  DataAssociation();
  if (DataAssociationDone != calib_step_) {
    ROS_WARN("[Refinement] Need status: DataAssociationDone.");
    return;
  }
  TicToc timer;
  timer.tic();

  traj_manager_->trajInitFromSurfel(surfel_association_, opt_time_offset_);
  calib_step_ = RefineDone;
}

bool LIinitializer::findAssociatedPose(
    int idx_in_data_reader, double timestamp, Eigen::Matrix4d& loam_pose){
  double diff_t_min = std::numeric_limits<double>::max();
  for(int i = -5; i < 5; ++i){
    int idx = i + idx_in_data_reader;
    if(idx < 0 || idx >= loam_poses_.size() - 1) continue;
    double time_diff = std::abs(loam_poses_[idx].first - timestamp);
    if(time_diff < diff_t_min){
      diff_t_min = time_diff;
      loam_pose = loam_poses_[idx].second;
    }
  }
  return diff_t_min < 0.02;//the interval of RS point cloud is about 0.1s
}

void LIinitializer::Mapping(bool relocalization) {
  bool update_map = true;
  if (relocalization) {
    lidar_odom_->clearOdomData();
    update_map = false;
  } else {
    scan_undistortion_->undistortScan();
    lidar_odom_ = std::make_shared<LiDAROdometry>(ndt_resolution_);
  }

  double last_scan_t = 0;
  CHECK(dataset_reader_->get_scan_data().size() 
    == dataset_reader_->get_scan_timestamps().size());
  for (int i = 0; i < dataset_reader_->get_scan_data().size(); ++i) {
    const auto& scan_raw = dataset_reader_->get_scan_data().at(i);
    double scan_t = dataset_reader_->get_scan_timestamps().at(i);
    double scan_pcl = pcl_conversions::fromPCL(scan_raw.header.stamp).toSec();
    // LOG(INFO)<< "scan_t "<<scan_t - loam_poses_[i].first;
    // LOG(INFO)<< "scan_pcl "<<scan_pcl - loam_poses_[i].first;
    auto iter = scan_undistortion_->get_scan_data().find(scan_raw.header.stamp);
    if (iter != scan_undistortion_->get_scan_data().end()) {
      if(simulation_){
        int64_t stamp = scan_t * 1e9;
        auto it_pose = loam_poses_map_.find(stamp);
        if(it_pose != loam_poses_map_.end()){
          lidar_odom_->feedScan(
            scan_t, iter->second, it_pose->second, update_map, true);
        }
      }else{
        Eigen::Matrix4d pose_predict;
        bool ok = findAssociatedPose(i, scan_t, pose_predict);
        lidar_odom_->feedScan(
          scan_t, iter->second, pose_predict, update_map, true);
      }
      
      last_scan_t = scan_t;
    }
  }
}

void LIinitializer::SaveOptimizedPCDMap(const std::string filepath){
  scan_undistortion_->undistortScanInMap();
  pcl::io::savePCDFileASCII(filepath, *(scan_undistortion_->get_map_cloud()));
}

void LIinitializer::ReprojectPointCloudToImage(){
  scan_undistortion_->undistortScanInMap();
  
  for (int j = 0; j < dataset_reader_->get_scan_data().size(); ++j) {
    const auto& scan_raw = dataset_reader_->get_scan_data().at(j);
    double scan_t = dataset_reader_->get_scan_timestamps().at(j);
    double scan_pcl = pcl_conversions::fromPCL(scan_raw.header.stamp).toSec();
    auto iter = scan_undistortion_->get_scan_data().find(scan_raw.header.stamp);

    if(iter == scan_undistortion_->get_scan_data().end()) continue;

    //first find the image with cloeset timestamp.
    double img_t = -1.0;
    int idx = -1;
    for(int i = 0; i < dataset_reader_->get_image_data().size(); ++i){
      img_t = dataset_reader_->get_image_data()[i].timestamp;
      if(img_t > scan_t && std::abs(img_t - scan_t) < 0.05){
        idx = i;
        break;
      }
    }
    
    if(idx != -1){
      // get LiDAR pose in Camera frame
      Eigen::Quaterniond q_LtoG, q_CtoG;
      Eigen::Vector3d p_LinG, p_CinG;
      traj_manager_->evaluateLidarPose(scan_t, q_LtoG, p_LinG);
      traj_manager_->evaluateCameraPose(img_t, q_CtoG, p_CinG);
      Eigen::Quaterniond q_LtoC = q_CtoG.conjugate() * q_LtoG;
      Eigen::Vector3d p_LinC = q_CtoG.conjugate() * (p_LinG - p_CinG);

      cv::Mat img_proj;
      dataset_reader_->get_image_data()[idx].image.copyTo(img_proj);
      cv::cvtColor(img_proj, img_proj, CV_GRAY2BGR);
      const auto camera = traj_manager_->getCameraModel();
      for(const auto& pt : iter->second->points){
        Eigen::Vector3d ept;
        ept << pt.x, pt.y, pt.z;
        Eigen::Vector3d pt_c = q_LtoC * ept + p_LinC;
        if(pt_c[2] < 0) continue;
        // reproject
        Eigen::Vector2d uv;
        camera->spaceToPlane(pt_c, uv);
        if(uv[0] < 0 || uv[1] < 0 || int(uv[0]) > camera->cols() - 1 
            || int(uv[1]) > camera->rows() - 1){
          continue;
        }
        img_proj.at<cv::Vec3b>(int(uv[1]), int(uv[0]))[0] = 0;
        img_proj.at<cv::Vec3b>(int(uv[1]), int(uv[0]))[1] = 255;
        img_proj.at<cv::Vec3b>(int(uv[1]), int(uv[0]))[2] = 0;
      }
      // cv::imshow("img_proj", img_proj);
      // cv::waitKey(0);
      cv::imwrite("/home/wz/Desktop/img/"+std::to_string(j)+".jpg", img_proj);
    }
  }
}

};//end namespace

int main(int argc, char **argv){
  ros::init(argc, argv, "LVI-Calibration");
  ros::start();

  ros::NodeHandle nh("~");

  std::string bag_file_path = "";
  std::string orb_folder = "";
  std::string loam_folder = "";
  CHECK(nh.getParam("bag_filename", bag_file_path)) 
    << "'bag_filename' must be assigned!";
  CHECK(nh.getParam("orb_folder", orb_folder)) 
    << "'orb_folder' must be assigned!";
  CHECK(nh.getParam("loam_folder", loam_folder)) 
    << "'loam_folder' must be assigned!";
  std::string file_path_noext = 
    bag_file_path.substr(0, bag_file_path.find_last_of("."));
  std::string file_dir = 
    file_path_noext.substr(0, bag_file_path.find_last_of("/")); 
  std::string file_stem = 
    file_path_noext.substr(bag_file_path.find_last_of("/")); 

  std::string orb_result_path = file_dir + "/" + orb_folder + "/" + file_stem + ".txt";
  std::string loam_result_path = file_dir + "/" + loam_folder + "/" + file_stem + ".txt";

  licalib::LIinitializer lvi_initializer(nh);
  lvi_initializer.LoadOrbResults(orb_result_path);
  lvi_initializer.ReadPoseGT(loam_result_path);
  lvi_initializer.InitVariables();

  //TODO: remove these codes, all data has already been read in DataReader
  rosbag::Bag bag;
  ros::Rate r(20);
  bag.open(bag_file_path, rosbag::bagmode::Read);
  for(rosbag::MessageInstance const m: rosbag::View(bag)){
    sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
    if (imu != NULL) {
      lvi_initializer.ImuCallback(imu);
    }
  }
  bag.close();
  
  auto start = system_clock::now();
  lvi_initializer.ComputeIntegrationForFrames();
  lvi_initializer.EstimateInitExtrinsicCI();
  lvi_initializer.EstimateInitExtrinsicLI();
  lvi_initializer.LCIoptimize();
  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  cout <<  "花费了" 
     << double(duration.count()) * microseconds::period::num / microseconds::period::den 
     << "秒" << endl;
  ros::shutdown();

  return 0;
}

