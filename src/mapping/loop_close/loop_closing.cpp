/*
 * @Author: ding.yin
 * @Date: 2022-11-09 19:50:41
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-09 20:55:03
 */

#include "mapping/loop_close/loop_closing.hpp"
#include "glog/logging.h"

#include "models/cloud_filter/voxel_filter.hpp"
#include "models/registration/ndt_registration.hpp"
#include "pcl/io/pcd_io.h"

namespace avp_mapping {

LoopClosing::LoopClosing(std::string work_dir) { initWithConfig(work_dir); }

bool LoopClosing::initWithConfig(std::string work_dir) {
  std::string node_path = work_dir + "/config/mapping/loop_closing.yaml";
  YAML::Node node = YAML::LoadFile(node_path);

  LOG(INFO) << " -------- Loop Closing Init ---------";
  initParam(node);
  initDataPath(node);
  initRegistration(registration_ptr_, node);
  initFilter("map", map_filter_ptr_, node);
  initFilter("scan", scan_filter_ptr_, node);
  return true;
}

bool LoopClosing::initParam(const YAML::Node &node) {
  extend_frame_num_ = node["extend_frame_num"].as<int>();
  loop_step_ = node["loop_step"].as<int>();
  diff_num_ = node["diff_num"].as<int>();
  detect_area_ = node["detect_area"].as<double>();
  fitness_score_limit_ = node["fitness_score_limit"].as<double>();
  return true;
}

bool LoopClosing::initDataPath(const YAML::Node &node) {
  std::string data_path = node["data_path"].as<std::string>();
  key_frame_path_ = data_path + "/slam_data/key_frames";
  return true;
}

bool LoopClosing::initRegistration(
    std::shared_ptr<RegistrationInterface> &reg_ptr, const YAML::Node &node) {
  std::string reg_method = node["registration_method"].as<std::string>();
  LOG(INFO) << "Registration Method for Loop Closing: " << reg_method;
  if (reg_method == "NDT") {
    reg_ptr = std::make_shared<NDTRegistration>(node[reg_method]);
  } else {
    LOG(ERROR) << "Not Implmented Registration Method: " << reg_method;
  }
  return true;
}

bool LoopClosing::initFilter(std::string filter_user,
                             std::shared_ptr<CloudFilterInterface> &filter_ptr,
                             const YAML::Node &node) {
  std::string filter_method = node[filter_user + "_filter"].as<std::string>();
  LOG(INFO) << filter_user << " is " << filter_method;
  if (filter_method == "voxel_filter") {
    filter_ptr =
        std::make_shared<VoxelFilter>(node[filter_method][filter_user]);
  } else {
    LOG(ERROR) << "Not Implmented Filter Method: " << filter_method;
  }
  return true;
}

bool LoopClosing::update(const KeyFrame kf, const KeyFrame reli_odom) {
  has_new_loop_pose_ = false;
  all_key_frames_.push_back(kf);
  all_key_reli_odoms_.push_back(reli_odom);

  int key_frame_index = 0;
  if (!detectNearestKeyFrame(key_frame_index)) {
    return false;
  }
  if (!cloudRegistration(key_frame_index)) {
    return false;
    ;
  }
  has_new_loop_pose_ = true;
  return true;
}

bool LoopClosing::detectNearestKeyFrame(int &key_frame_index) {
  static int skip_cnt = 0;
  static int skip_num = loop_step_;
  if (++skip_cnt < skip_num)
    return false;

  if (static_cast<int>(all_key_reli_odoms_.size()) < diff_num_ + 1)
    return false;

  int key_num = static_cast<int>(all_key_reli_odoms_.size());
  double min_distance = 100000.0;
  double distance = 0.0;

  KeyFrame history_key_frame;
  KeyFrame current_key_frame = all_key_reli_odoms_.back();

  for (int i = 0; i < key_num - 1; ++i) {
    if (key_num - i < diff_num_)
      break;
    history_key_frame = all_key_reli_odoms_.at(i);
    distance =
        fabs(current_key_frame.pose(0, 3) - history_key_frame.pose(0, 3)) +
        fabs(current_key_frame.pose(0, 3) - history_key_frame.pose(0, 3)) +
        fabs(current_key_frame.pose(0, 3) - history_key_frame.pose(0, 3));

    if (distance < min_distance) {
      min_distance = distance;
      key_frame_index = i;
    }
  }

  if (key_frame_index < extend_frame_num_) {
    return false;
  }

  skip_cnt = 0;
  skip_num = static_cast<int>(min_distance);
  if (min_distance > detect_area_) {
    skip_num = std::max(loop_step_, static_cast<int>(min_distance / 2.0));
    return false;
  } else {
    skip_num = loop_step_;
    return true;
  }
}

bool LoopClosing::cloudRegistration(int key_frame_index) {
  // multi frame joint
  CloudData::CLOUD_PTR map_cloud_ptr(new CloudData::CLOUD());
  Eigen::Matrix4d map_pose = Eigen::Matrix4d::Identity();
  jointMap(key_frame_index, map_cloud_ptr, map_pose);
  // scan joint
  CloudData::CLOUD_PTR scan_cloud_ptr(new CloudData::CLOUD());
  Eigen::Matrix4d scan_pose = Eigen::Matrix4d::Identity();
  jointScan(scan_cloud_ptr, scan_pose);

  // registration
  Eigen::Matrix4d result_pose = Eigen::Matrix4d::Identity();
  registration(map_cloud_ptr, scan_cloud_ptr, scan_pose, result_pose);

  if (registration_ptr_->getFitnessScore() > fitness_score_limit_) {
    return false;
  }

  static int loop_close_cnt = 0;
  loop_close_cnt++;

  LOG(INFO) << loop_close_cnt << " Loop Closure Detected " << std::endl
            << curr_loop_pose_.index0 << " -----> " << curr_loop_pose_.index1
            << std::endl
            << "fitness score: " << registration_ptr_->getFitnessScore()
            << std::endl;

  return true;
}

bool LoopClosing::jointMap(int key_frame_index,
                           CloudData::CLOUD_PTR &map_cloud_ptr,
                           Eigen::Matrix4d &map_pose) {
  map_pose = all_key_reli_odoms_.at(key_frame_index).pose;
  curr_loop_pose_.index0 = all_key_frames_.at(key_frame_index).index;

  Eigen::Matrix4d pose_to_reli_odom =
      map_pose * all_key_frames_.at(key_frame_index).pose.inverse();

  for (int i = key_frame_index - extend_frame_num_;
       i < key_frame_index + extend_frame_num_; ++i) {
    std::string file_path = key_frame_path_ + "/key_frame_" +
                            std::to_string(all_key_frames_.at(i).index) +
                            ".pcd";
    CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
    pcl::io::loadPCDFile(file_path, *cloud_ptr);

    Eigen::Matrix4d cloud_pose = pose_to_reli_odom * all_key_frames_.at(i).pose;
    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, cloud_pose);
    *map_cloud_ptr += *cloud_ptr;
  }
  map_filter_ptr_->filter(map_cloud_ptr, map_cloud_ptr);
  return true;
}

bool LoopClosing::jointScan(CloudData::CLOUD_PTR &scan_cloud_ptr,
                            Eigen::Matrix4d &scan_pose) {
  scan_pose = all_key_reli_odoms_.back().pose;
  curr_loop_pose_.index1 = all_key_reli_odoms_.back().index;
  curr_loop_pose_.time = all_key_reli_odoms_.back().time;

  std::string file_path = key_frame_path_ + "/key_frame_" +
                          std::to_string(curr_loop_pose_.index1) + ".pcd";
  pcl::io::loadPCDFile(file_path, *scan_cloud_ptr);
  scan_filter_ptr_->filter(scan_cloud_ptr, scan_cloud_ptr);
  return true;
}

bool LoopClosing::registration(CloudData::CLOUD_PTR &map_cloud_ptr,
                               CloudData::CLOUD_PTR &scan_cloud_ptr,
                               Eigen::Matrix4d &scan_pose,
                               Eigen::Matrix4d &result_pose) {
  CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
  registration_ptr_->setInputTarget(map_cloud_ptr);
  registration_ptr_->scanMatch(scan_cloud_ptr, scan_pose, result_cloud_ptr,
                               result_pose);

  return true;
}

bool LoopClosing::hasNewLoopPose() { return has_new_loop_pose_; }

LoopPose &LoopClosing::getCurrentLoopPose() { return curr_loop_pose_; }

} // namespace avp_mapping