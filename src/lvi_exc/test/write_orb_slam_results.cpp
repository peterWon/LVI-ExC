#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
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
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include "sensor_msgs/PointCloud2.h"

//ORB-SLAM
#include "System.h"
#include "Converter.h"

#include <Eigen/Core>
#include <Eigen/Dense>
// #include <opencv2/core/eigen.hpp>
#include <glog/logging.h>

class OrbSlamWriter{
public:
  explicit OrbSlamWriter(const std::string& orb_voc, 
      const std::string& orb_slam_cfg){
    // Create SLAM system. 
    // It initializes all system threads and gets ready to process frames.
    orb_slam_ = std::make_shared<ORB_SLAM2::System>(
      orb_voc, orb_slam_cfg, ORB_SLAM2::System::MONOCULAR, true); 
  }

  ~OrbSlamWriter(){
    if(orb_slam_){
      // Stop all threads
      orb_slam_->Shutdown();      
    }
  }
  
  void ImgCallback(const sensor_msgs::ImageConstPtr& img_msg);
  void Shutdown(){orb_slam_->Shutdown();}
  void WriteObservations(const std::string& map_out_path);
private:
  //可以通过MapPoints来顺次添加观测到trajectory
  //通过KeyFrame来获取所有关键帧的位姿 & 时间戳
  std::shared_ptr<ORB_SLAM2::System> orb_slam_;
};

void OrbSlamWriter::ImgCallback(const sensor_msgs::ImageConstPtr &img_msg){
  cv_bridge::CvImageConstPtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvShare(img_msg);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if(cv_ptr->image.channels()==3){
    cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2GRAY);
  }
  cv::Mat Tcw = orb_slam_->TrackMonocular(
    cv_ptr->image, cv_ptr->header.stamp.toSec());
  
  // 是否需要记录每一帧的位姿及关键点
  // std::vector<ORB_SLAM2::MapPoint*> map_pts = orb_slam_->GetTrackedMapPoints();
  // std::vector<cv::KeyPoint> key_pts = orb_slam_->GetTrackedKeyPointsUn();
  // LOG(INFO)<<"Map size and keypoint size: "<<map_pts.size()<<", "<<key_pts.size();
}

void OrbSlamWriter::WriteObservations(const std::string& map_out_path){
  std::ofstream ofs(map_out_path);
  if(!ofs.is_open()){
    LOG(ERROR)<<"Open file: "<<map_out_path<<" failed!";
    return;
  }
  
  //Write All Frame Poses
  /***************************************************
   * KF FrameID t q
   * U V LandmarkID, U V LandmarkID,...
   * ...
   **************************************************/
  vector<ORB_SLAM2::KeyFrame*> vpKFs = orb_slam_->GetMap()->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  // cv::Mat Two = vpKFs[0]->GetPoseInverse();
  cv::Mat Two = cv::Mat::eye(4,4,CV_32F);
  // f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
  // We need to get first the keyframe pose and then concatenate the relative transformation.
  // Frames not localized (tracking failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
  // which is true when tracking failed (lbL).
  /* auto* mpTracker = orb_slam_->GetTracker();
  list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();
  for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
      lend = mpTracker->mlRelativeFramePoses.end();
      lit!=lend;lit++, lRit++, lT++, lbL++){
    if(*lbL)
      continue;

    ORB_SLAM2::KeyFrame* pKF = *lRit;

    cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    while(pKF->isBad()){
      Trw = Trw*pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw*pKF->GetPose()*Two;

    cv::Mat Tcw = (*lit)*Trw;
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

    int64_t frame_id = *lT * 1e9;
    ofs << "FramePose "<< frame_id
        << setprecision(7) << " " 
        << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2)
        << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "\n";
  } */
  
  //Write Keyframe UV points
  for(size_t i = 0; i < vpKFs.size(); i++){
    ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
    int64_t frame_id = pKF->mTimeStamp * 1e9;
    if(pKF->isBad())
      continue;

    cv::Mat R = pKF->GetRotation().t();
    std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
    cv::Mat twc = pKF->GetCameraCenter();
    ofs << "FramePose "<< frame_id
        << setprecision(7) << " " 
        << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2)
        << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "\n";

    ofs << "UV "<< frame_id <<" ";
    const auto& mappts = pKF->GetMapPointMatches();
    for(size_t i = 0; i < mappts.size(); ++i){
      const auto& mappt =  mappts[i];
      if(!mappt || mappt->isBad()) continue;
      const auto& uv = pKF->mvKeysUn[i];
      ofs << uv.pt.x<<" "<<uv.pt.y<<" "<<mappt->mnId<<" ";
    }
    ofs<<"\n"; 
  }

  //Write MapPoints
  /************************
   * MP-LandmarkID x y z ReferenceFrameID
   * MP-LandmarkID x y z
   * ...
  ************************/
  auto mappts = orb_slam_->GetMap()->GetAllMapPoints();
  std::sort(mappts.begin(), mappts.end(), ORB_SLAM2::MapPoint::lId);
  for(const auto& pt: mappts){
    if(pt->GetReferenceKeyFrame()->isBad()) continue;
    cv::Mat Tcw = pt->GetReferenceKeyFrame()->GetPose();//in camera frame
    int64_t refid = pt->GetReferenceKeyFrame()->mTimeStamp * 1e9;

    const auto R = Tcw.rowRange(0,3).colRange(0,3);
    const auto t = Tcw.rowRange(0,3).col(3);
    const cv::Mat p_in_camera = R * pt->GetWorldPos() + t;//transform to camera frame
 
    ofs << "MapPoint " <<pt->mnId 
        << " " << p_in_camera.at<float>(0)
        << " " << p_in_camera.at<float>(1)
        << " " << p_in_camera.at<float>(2)
        << " " << refid << "\n";
  }

  ofs.close();
  LOG(INFO)<<"Write key frames and map points succeed!";
}

int main(int argc, char **argv){
  ros::init(argc, argv, "OrbSlamWriter");
  ros::start();
  
  ros::NodeHandle nh("~");

  std::string orb_voc="";
  std::string orb_slam_cfg="";
  std::string image_topic="";
  CHECK(nh.getParam("orb_voc", orb_voc)) << "'orb_voc' must be assigned!";
  CHECK(nh.getParam("orb_slam_cfg", orb_slam_cfg))
    << "'orb_slam_cfg' must be assigned!";
  CHECK(nh.getParam("image_topic", image_topic)) << "'image_topic' must be assigned!";
  
  std::string bag_file_path = "";
  std::string orb_folder = "";
  CHECK(nh.getParam("bag_filename", bag_file_path)) 
    << "'bag_filename' must be assigned!";
  CHECK(nh.getParam("orb_folder", orb_folder)) 
    << "'orb_folder' must be assigned!";
  std::string file_path_noext = 
    bag_file_path.substr(0, bag_file_path.find_last_of("."));
  std::string file_dir = 
    file_path_noext.substr(0, bag_file_path.find_last_of("/")); 
  std::string file_stem = 
    file_path_noext.substr(bag_file_path.find_last_of("/")); 

  std::string orb_result_path = file_dir + "/" + orb_folder + "/" + file_stem + ".txt";

  double bag_start = 0;
  double bag_durr = 0;
  double te = 0;
  CHECK(nh.getParam("bag_start", bag_start));
  CHECK(nh.getParam("bag_durr", bag_durr));
  
  OrbSlamWriter vslam_writer(orb_voc, orb_slam_cfg);

  rosbag::Bag bag;
  bag.open(bag_file_path, rosbag::bagmode::Read);
  
  rosbag::View view;
  std::vector<std::string> topics;
  topics.push_back(image_topic);

  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(bag_start);
  ros::Time time_finish = (bag_durr < 0)?
      view_full.getEndTime() : time_init + ros::Duration(bag_durr);
  view.addQuery(bag, rosbag::TopicQuery(topics), time_init, time_finish);

  for(rosbag::MessageInstance const m: view){
    sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
    if (img != NULL) {
      vslam_writer.ImgCallback(img);
    }
  }
  bag.close();
  vslam_writer.WriteObservations(orb_result_path);
  vslam_writer.Shutdown();
  ros::shutdown();

  return 0;
}

