#include "vi_init/parameters.h"
#include <opencv2/core/eigen.hpp>
std::string CONFIG_FILE;
std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::string OUTPUT_PATH;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
std::string BAG_FILENAME;
int MAX_CNT;
int MIN_DIST;
int FREQ;
int WINDOW_SIZE;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
double TD, TR;

int TAG_TYPE;
double TAG_SIZE;
double TAG_SPACE;
int BLACK_BORDER;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name){
  T ans;
  if (n.getParam(name, ans)){
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  }else{
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

void readParameters(ros::NodeHandle &n){
  CONFIG_FILE = readParam<std::string>(n, "config_file");
  cv::FileStorage fsSettings(CONFIG_FILE, cv::FileStorage::READ);
  if(!fsSettings.isOpened()){
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }
  std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");
  
  
  fsSettings["image_topic"] >> IMAGE_TOPIC;
  fsSettings["imu_topic"] >> IMU_TOPIC;
  fsSettings["imu_topic"] >> OUTPUT_PATH;
  fsSettings["bag_filename"] >> BAG_FILENAME;

  WINDOW_SIZE = fsSettings["window_size"];
  SOLVER_TIME = fsSettings["max_solver_time"];
  NUM_ITERATIONS = fsSettings["max_num_iterations"];
  MIN_PARALLAX = fsSettings["keyframe_parallax"];
  MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;
  ACC_N = fsSettings["acc_n"];
  ACC_W = fsSettings["acc_w"];
  GYR_N = fsSettings["gyr_n"];
  GYR_W = fsSettings["gyr_w"];
  G.z() = fsSettings["g_norm"];
  INIT_DEPTH = 5.0;
  BIAS_ACC_THRESHOLD = 0.1;
  BIAS_GYR_THRESHOLD = 0.1;
  TD = fsSettings["td"];

  MAX_CNT = fsSettings["max_cnt"];
  MIN_DIST = fsSettings["min_dist"];
  ROW = fsSettings["image_height"];
  COL = fsSettings["image_width"];
  FREQ = fsSettings["freq"];
  F_THRESHOLD = fsSettings["F_threshold"];
  SHOW_TRACK = fsSettings["show_track"];
  EQUALIZE = fsSettings["equalize"];
  FISHEYE = fsSettings["fisheye"];
  
  if (FISHEYE == 1)
    FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
  CAM_NAMES.push_back(CONFIG_FILE);
  
  cv::Mat cv_R, cv_T;
  fsSettings["extrinsicRotation"] >> cv_R;
  fsSettings["extrinsicTranslation"] >> cv_T;
  Eigen::Matrix3d eigen_R;
  Eigen::Vector3d eigen_T;
  cv::cv2eigen(cv_R, eigen_R);
  cv::cv2eigen(cv_T, eigen_T);
  Eigen::Quaterniond Q(eigen_R);
  eigen_R = Q.normalized();
  RIC.push_back(eigen_R);
  TIC.push_back(eigen_T);


  STEREO_TRACK = false;
  FOCAL_LENGTH = 460;
  PUB_THIS_FRAME = false;

  TAG_SIZE = fsSettings["tag_size"];
  TAG_TYPE = fsSettings["tag_type"];
  TAG_SPACE = fsSettings["tag_space"];
  BLACK_BORDER = fsSettings["black_border"];

  if (FREQ == 0)
    FREQ = 100;

  fsSettings.release();
}
