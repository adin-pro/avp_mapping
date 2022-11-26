/*
 * @Author: ding.yin
 * @Date: 2022-11-07 20:26:42
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-25 16:50:37
 */

#include "mapping/back_end/back_end_flow.hpp"

namespace avp_mapping {

BackEndFlow::BackEndFlow(ros::NodeHandle &nh, std::string work_dir) {
  YAML::Node node = YAML::LoadFile(work_dir + "/config/mapping/back_end.yaml");
  // subscriber
  key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(
      nh, node["key_frame_sub_topic"].as<std::string>(), 1000);

  loop_pose_sub_ptr_ = std::make_shared<LoopPoseSubscriber>(
      nh, node["loop_pose_sub_topic"].as<std::string>(), 1000);
  // publisher
  key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(
      nh, node["key_frames_pub_topic"].as<std::string>(),
      node["frame_id"].as<std::string>(), 1000);

  pgo_back_end_ptr_ = std::make_shared<PGOBackEnd>(node);

  odom_info_ = node["odom_info"].as<std::vector<double>>();
  loop_pose_info_ = node["loop_pose_info"].as<std::vector<double>>();
}

bool BackEndFlow::run() {
  if (!readData())
    return false;
  if (hasData()) {
    maybeInsertLoopPose();
    MaybeInsertKFOdometry();
    publishData();
  }
  return true;
}

bool BackEndFlow::readData() {
  key_frame_sub_ptr_->parseData(key_frame_buff_);
  loop_pose_sub_ptr_->parseData(loop_pose_buff_);
  return true;
}

bool BackEndFlow::hasData() {
  if (key_frame_buff_.size() == 0 && loop_pose_buff_.size() == 0)
    return false;
  return true;
}

bool BackEndFlow::maybeInsertLoopPose() {
  if (loop_pose_buff_.size() == 0)
    return false;
  while (loop_pose_buff_.size() > 0) {
    curr_loop_pose_ = loop_pose_buff_.front();
    pgo_back_end_ptr_->AddConstraint(curr_loop_pose_.index1,
                                     curr_loop_pose_.index0,
                                     curr_loop_pose_.pose, loop_pose_info_);
    LOG(INFO) << "Add Loop Pose  Constraint " << curr_loop_pose_.index0
              << " ----> " << curr_loop_pose_.index1;
    std::cout << curr_loop_pose_.pose << std::endl;
    loop_pose_buff_.pop_front();
  }
  pgo_back_end_ptr_->Optimize();
  return true;
}

bool BackEndFlow::MaybeInsertKFOdometry() {
  static bool inited = false;
  if (key_frame_buff_.size() == 0) {
    return false;
  }

  if (!inited) {
    inited = true;
    last_key_frame_ = key_frame_buff_.front();
    curr_key_frame_ = key_frame_buff_.front();
    pgo_back_end_ptr_->AddPose(curr_key_frame_);
    LOG(INFO) << "Add Pose " << curr_key_frame_.index << "  (x,y) "
              << "(" << curr_key_frame_.pose(0, 3) << ", "
              << curr_key_frame_.pose(1, 3) << ")";
    key_frame_buff_.pop_front();
    return true;
  }

  while (key_frame_buff_.size() > 0) {
    curr_key_frame_ = key_frame_buff_.front();
    KeyFrame last_optimized_kf = optimized_kfs_.back();
    // odom from last to current
    Eigen::Matrix4d T_be =
        curr_key_frame_.pose.inverse() * last_key_frame_.pose;
    // optimized pose
    KeyFrame optimized_curr_kf = curr_key_frame_;
    optimized_curr_kf.pose = last_optimized_kf.pose * T_be.inverse();
    pgo_back_end_ptr_->AddPose(optimized_curr_kf);
    LOG(INFO) << "Add Pose " << curr_key_frame_.index << "  (x,y) "
              << "(" << curr_key_frame_.pose(0, 3) << ", "
              << curr_key_frame_.pose(1, 3) << ")";

    pgo_back_end_ptr_->AddConstraint(curr_key_frame_.index,
                                     last_key_frame_.index, T_be, odom_info_);
    LOG(INFO) << "Add Odometry Constraint " << last_key_frame_.index
              << " ----> " << curr_key_frame_.index;
    std::cout << T_be << std::endl;
    last_key_frame_ = curr_key_frame_;
    key_frame_buff_.pop_front();
  }
  return true;
}

bool BackEndFlow::publishData() {
  pgo_back_end_ptr_->GetOptimizedPoses(optimized_kfs_);
  key_frames_pub_ptr_->publish(optimized_kfs_);
  return true;
}

} // namespace avp_mapping
