/*
 * @Author: ding.yin
 * @Date: 2022-11-07 16:03:29
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-08 21:11:07
 */

#include "mapping/back_end/back_end.hpp"
#include "models/optimizer/g2o/g2o_optimizer.hpp"

#include "glog/logging.h"
#include "pcl/io/pcd_io.h"
#include "yaml-cpp/yaml.h"

namespace avp_mapping {

BackEnd::BackEnd(std::string &work_dir) { initWithConfig(work_dir); }

bool BackEnd::initWithConfig(std::string &work_dir) {
  std::string config_file_path = work_dir + "/config/mapping/back_end.yaml";
  YAML::Node node = YAML::LoadFile(config_file_path);
  LOG(INFO) << " ------------ Init Back End ------------";
  initParam(node);
  initGraphOptimizer(node);
  initDataPath(node);
  return true;
}

bool BackEnd::initParam(const YAML::Node &config_node) {
  key_frame_dist_ = config_node["key_frame_distance"].as<double>();
  return true;
}

bool BackEnd::initGraphOptimizer(const YAML::Node &config_node) {
  std::string graph_optimizer_type =
      config_node["graph_optimizer_type"].as<std::string>();
  if (graph_optimizer_type == "g2o") {
    optimizer_ptr_ = std::make_shared<G2OGraphOptimizer>("lm_var");
  } else {
    LOG(ERROR) << "Invalid Optimizer Type: " << graph_optimizer_type;
    return false;
  }

  LOG(INFO) << "Optimizer Type: " << graph_optimizer_type;
  graph_optimizer_config_.use_reliable_odom =
      config_node["use_reliable_odom"].as<bool>();
  graph_optimizer_config_.use_loop_close =
      config_node["use_loop_close"].as<bool>();

  graph_optimizer_config_.optimize_step_with_reliable_odom =
      config_node["optimize_step_with_reliable_odom"].as<int>();
  graph_optimizer_config_.optimize_step_with_loop =
      config_node["optimize_step_with_reliable_odom"].as<int>();
  graph_optimizer_config_.optimize_step_with_key_frame =
      config_node["optimize_step_with_key_frame"].as<int>();

  // 6 DOF noise
  for (int i = 0; i < 6; ++i) {
    graph_optimizer_config_.vidar_odom_edge_noise(i) =
        config_node[graph_optimizer_type + "_param"]["odom_edge_noise"][i]
            .as<double>();
    graph_optimizer_config_.close_loop_noise(i) =
        config_node[graph_optimizer_type + "_param"]["close_loop_noise"][i]
            .as<double>();
  }

  for (int i = 0; i < 3; ++i) {
    graph_optimizer_config_.reliable_odom_noise(i) =
        config_node[graph_optimizer_type + "_param"]["reliable_odom_noise"][i]
            .as<double>();
  }

  return true;
}

// TODO FILE_MANAGER
bool BackEnd::initDataPath(const YAML::Node &config_node) {
  std::string data_path = config_node["data_path"].as<std::string>();

  key_frame_path_ = data_path + "/slam_data/key_frames";
  traj_path_ = data_path + "/slam_data/trajectory";

  return true;
}

bool BackEnd::update(const CloudData &cloud_data, const PoseData &vidar_odom,
                     const PoseData &reliable_odom) {
  resetParam();
  if (needNewKeyFrame(cloud_data, vidar_odom, reliable_odom)) {
    savePose(ground_truth_ofs_, reliable_odom.pose);
    savePose(vidar_odom_ofs_, vidar_odom.pose);
    addNodeAndEdge(reliable_odom);
    if (needOptimization()) {
      saveOptimizedPose();
    }
  }
  return true;
}

bool BackEnd::insertLoopPose(const LoopPose &loop_pose) {

  if (!graph_optimizer_config_.use_loop_close) {
    return false;
  }

  Eigen::Isometry3d isom;
  isom.matrix() = loop_pose.pose;
  optimizer_ptr_->addSE3Edge(loop_pose.index0, loop_pose.index1, isom,
                             graph_optimizer_config_.close_loop_noise);
  loop_pose_cnt_++;
  return true;
}

void BackEnd::resetParam() {
  has_new_key_frame_ = false;
  has_new_optimized_ = false;
}

bool BackEnd::savePose(std::ofstream &ofs, const Eigen::Matrix4d &pose) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << pose(i, j);
      if (i == 2 && j == 3) {
        ofs << std::endl;
      } else {
        ofs << " ";
      }
    }
  }
  return true;
}

bool BackEnd::needNewKeyFrame(const CloudData &cloud_data,
                              const PoseData &vidar_odom,
                              const PoseData &reliable_odom) {
  static Eigen::Matrix4d last_key_pose = vidar_odom.pose;
  if (key_frames_deque_.size() == 0) {
    has_new_key_frame_ = true;
    last_key_pose = vidar_odom.pose;
  }

  if (fabs(vidar_odom.pose(0, 3) - last_key_pose(0, 3)) +
          fabs(vidar_odom.pose(1, 3) - last_key_pose(1, 3)) +
          fabs(vidar_odom.pose(2, 3) - last_key_pose(2, 3)) >
      key_frame_dist_) {
    has_new_key_frame_ = true;
    last_key_pose = vidar_odom.pose;
  }

  if (has_new_key_frame_) {
    // std::string pcd_file_path = key_frame_path_ + "/key_frame" +
    // std::to_string(key_frames_deque_.size()) + ".pcd";
    // pcl::io::savePCDFileBinary(pcd_file_path, *cloud_data.cloud_ptr);

    KeyFrame kf;
    kf.time = vidar_odom.time;
    kf.index = static_cast<int>(key_frames_deque_.size());
    kf.pose = vidar_odom.pose;
    key_frames_deque_.push_back(kf);
    current_key_frame_ = kf;

    current_key_reliable_odom_.time = reliable_odom.time;
    current_key_reliable_odom_.index = kf.index;
    current_key_reliable_odom_.pose = reliable_odom.pose;
  }

  return has_new_key_frame_;
}

bool BackEnd::addNodeAndEdge(const PoseData &reliable_odom) {

  Eigen::Isometry3d isom;
  isom.matrix() = reliable_odom.pose;
  if (!graph_optimizer_config_.use_reliable_odom &&
      optimizer_ptr_->getNodeNum() == 0) {
    // fixed node only
    optimizer_ptr_->addSE3Node(isom, true);
  } else {
    optimizer_ptr_->addSE3Node(isom, false);
  }
  key_frame_cnt_++;

  // vidar odom edge
  static KeyFrame last_key_frame = current_key_frame_;
  int node_num = optimizer_ptr_->getNodeNum();
  if (node_num > 1) {
    Eigen::Matrix4d relative_pose =
        last_key_frame.pose.inverse() * current_key_frame_.pose;
    isom.matrix() = relative_pose;
    optimizer_ptr_->addSE3Edge(node_num - 2, node_num - 1, isom,
                               graph_optimizer_config_.vidar_odom_edge_noise);
  }

  // reliable odom edge as prior information
  if (graph_optimizer_config_.use_reliable_odom) {
    Eigen::Vector3d xyz(reliable_odom.pose(0, 3), reliable_odom.pose(1, 3),
                        reliable_odom.pose(2, 3));
    optimizer_ptr_->addSE3PriorXYZEdge(
        node_num - 1, xyz, graph_optimizer_config_.reliable_odom_noise);
    reliable_odom_cnt_++;
  }

  return true;
}

bool BackEnd::needOptimization() {
  if (reliable_odom_cnt_ >=
          graph_optimizer_config_.optimize_step_with_reliable_odom ||
      loop_pose_cnt_ >= graph_optimizer_config_.optimize_step_with_loop ||
      key_frame_cnt_ >= graph_optimizer_config_.optimize_step_with_key_frame) {

    reliable_odom_cnt_ = 0;
    loop_pose_cnt_ = 0;
    key_frame_cnt_ = 0;

    if (optimizer_ptr_->optimize())
      has_new_optimized_ = true;
    return true;
  }

  return false;
}

// TODO
bool BackEnd::saveOptimizedPose() {
  optimizer_ptr_->getOptimizedPose(optimized_pose_);
  return true;
}

bool BackEnd::forceOptimize() {
  if (optimizer_ptr_->optimize())
    has_new_optimized_ = true;
  saveOptimizedPose();
  return has_new_optimized_;
}

bool BackEnd::getOptimizedKFs(std::deque<KeyFrame> &key_frames_deque) {
  KeyFrame kf;
  for (size_t i = 0; i < optimized_pose_.size(); ++i) {
    kf.pose = optimized_pose_.at(i);
    kf.index = static_cast<int>(i);
    key_frames_deque.push_back(kf);
  }
  return true;
}

bool BackEnd::hasNewKeyFrame() { return has_new_key_frame_; }

bool BackEnd::hasNewOptimized() { return has_new_optimized_; }

void BackEnd::getLatesetKeyFrame(KeyFrame &kf) { kf = current_key_frame_; }

void BackEnd::getLatesetKeyReliableOdom(KeyFrame &kf) {
  kf = current_key_reliable_odom_;
}

} // namespace avp_mapping