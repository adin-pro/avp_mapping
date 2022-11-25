/*
 * @Author: ding.yin
 * @Date: 2022-11-09 20:37:37
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-24 22:15:20
 */

#include "mapping/loop_close/loop_closing_flow.hpp"
#include "tools/file_manager.hpp"

namespace avp_mapping {
LoopClosingFlow::LoopClosingFlow(ros::NodeHandle &nh, std::string work_dir) {
  YAML::Node node =
      YAML::LoadFile(work_dir + "/config/mapping/loop_closing.yaml");
  // subscribers
  key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(
      nh, node["kf_sub_topic"].as<std::string>(), 1000);
  kf_cloud_height_sub_ptr_ = std::make_shared<CloudSubscriber>(
      nh, node["kf_cloud_height_sub_topic"].as<std::string>(), 1000);
  kf_image_sub_ptr_ = std::make_shared<ImageSubscriber>(
      nh, node["kf_image_sub_topic"].as<std::string>(), 1000);
  // publishers
  loop_pose_pub_ptr_ = std::make_shared<LoopPosePublisher>(
      nh, node["loop_pose_pub_topic"].as<std::string>(),
      node["frame_id"].as<std::string>(), 100);
  // loop_closing
  loop_closing_ptr_ = std::make_shared<SemanticLoopClosing>(node);
  // save
  save_dir_ = node["save_dir"].as<std::string>();
  FileManager::CreateDirectory(save_dir_);
  save_loop_pose_ = node["save_loop_pose"].as<bool>();
  loop_pose_fd_.open(save_dir_ + "/loop_pose.txt", std::ios::out);
}

bool LoopClosingFlow::run() {
  if (!readData())
    return false;
  if (hasData() && validData()) {
    if (loop_closing_ptr_->TryGetLoopPose(curr_image_data_,
                                          curr_cloud_height_data_,
                                          curr_loop_pose_, curr_key_frame_))
      publishData();
  }
  return true;
}

bool LoopClosingFlow::readData() {
  kf_cloud_height_sub_ptr_->parseData(cloud_data_buff_);
  kf_image_sub_ptr_->parseData(image_data_buff_);
  key_frame_sub_ptr_->parseData(key_frame_buff_);
  return true;
}

bool LoopClosingFlow::hasData() {
  if (cloud_data_buff_.size() == 0) {
    return false;
  }
  if (image_data_buff_.size() == 0) {
    return false;
  }
  if (key_frame_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool LoopClosingFlow::validData() {
  curr_cloud_height_data_ = cloud_data_buff_.front();
  cloud_data_buff_.pop_front();
  ImageData::getImageDataByTS(image_data_buff_, curr_cloud_height_data_.time,
                              curr_image_data_);
  KeyFrame::getKFDataByTS(key_frame_buff_, curr_cloud_height_data_.time,
                          curr_key_frame_);
  LOG(INFO) << "Received KF " << curr_key_frame_.time << " "
            << curr_key_frame_.index << " x " << curr_key_frame_.pose(0, 3)
            << " y " << curr_key_frame_.pose(1, 3);
  return true;
}

bool LoopClosingFlow::publishData() {
  loop_pose_pub_ptr_->publish(curr_loop_pose_);
  if (save_loop_pose_) {
    Eigen::Quaterniond q;
    q = curr_loop_pose_.pose.block<3, 3>(0, 0);
    loop_pose_fd_ << curr_loop_pose_.index0 << " " << curr_loop_pose_.index1
                  << " " << curr_loop_pose_.pose(0, 3) << " "
                  << curr_loop_pose_.pose(1, 3) << " "
                  << curr_loop_pose_.pose(2, 3) << " " << q.x() << " " << q.y()
                  << " " << q.z() << " " << q.w() << std::endl;
    loop_pose_fd_.flush();
  }

  return true;
}

} // namespace avp_mapping