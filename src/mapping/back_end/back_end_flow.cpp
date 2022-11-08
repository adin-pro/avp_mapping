/*
 * @Author: ding.yin
 * @Date: 2022-11-07 20:26:42
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-07 21:27:01
 */

#include "mapping/back_end/back_end_flow.hpp"

namespace avp_mapping {

BackEndFlow::BackEndFlow(ros::NodeHandle &nh, std::string cloud_topic,
                         std::string vidar_odom_topic, std::string work_dir) {
  // subscriber
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 10000);
  odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/odom", 10000);
  vidar_odom_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, vidar_odom_topic, 10000);
  loop_pose_sub_ptr_ =
      std::make_shared<LoopPoseSubscriber>(nh, "/loop_pose", 10000);
  // publisher
  transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/transformed_odom", "/base_link", "/vidar", 100);
  key_frame_pub_ptr_ =
      std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/base_link", 100);
  key_reliable_odom_pub_ptr_ = std::make_shared<KeyFramePublisher>(
      nh, "/key_reliable_odom", "/base_link", 100);
  key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(
      nh, "optimized_kfs", "/base_link", 100);

  back_end_ptr_ = std::make_shared<BackEnd>(work_dir);
}

bool BackEndFlow::run() {
  if (!readData())
    return false;

  maybeInsertLoopPose();
  while (hasData()) {
    if (!validData())
      continue;
    udpateBackEnd();
    publishData();
  }
  return true;
}

bool BackEndFlow::forceOptimize() {
  back_end_ptr_->forceOptimize();
  if (back_end_ptr_->hasNewOptimized()) {
    std::deque<KeyFrame> optimized_key_frames;
    back_end_ptr_->getOptimizedKFs(optimized_key_frames);
    key_frames_pub_ptr_->publish(optimized_key_frames);
  }
  return true;
}

bool BackEndFlow::readData() {
  cloud_sub_ptr_->parseData(cloud_data_buff_);
  odom_sub_ptr_->parseData(reli_odom_buff_);
  vidar_odom_sub_ptr_->parseData(vidar_odom_buff_);
  loop_pose_sub_ptr_->parseData(loop_pose_buff_);
  return true;
}

bool BackEndFlow::maybeInsertLoopPose() {
  while (loop_pose_buff_.size() > 0) {
    back_end_ptr_->insertLoopPose(loop_pose_buff_.front());
    loop_pose_buff_.pop_front();
  }
  return true;
}

bool BackEndFlow::hasData() {
  if (cloud_data_buff_.size() == 0)
    return false;
  if (vidar_odom_buff_.size() == 0)
    return false;
  if (reli_odom_buff_.size() == 0)
    return false;
  return true;
}

bool BackEndFlow::validData() {
  curr_cloud_data_ = cloud_data_buff_.front();
  curr_vidar_odom_ = vidar_odom_buff_.front();
  curr_reli_odom_ = reli_odom_buff_.front();

  double diff_vidar_time = curr_cloud_data_.time - curr_vidar_odom_.time;
  double diff_odom_time = curr_cloud_data_.time - curr_reli_odom_.time;

  if (diff_vidar_time < -0.03 || diff_odom_time < -0.03) {
    cloud_data_buff_.pop_front();
    return false;
  }
  if (diff_odom_time > 0.03) {
    reli_odom_buff_.pop_front();
    return false;
  }
  if (diff_vidar_time > 0.03) {
    vidar_odom_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  vidar_odom_buff_.pop_front();
  reli_odom_buff_.pop_front();
  return true;
}

bool BackEndFlow::udpateBackEnd() {
  static bool odom_inited = false;
  static Eigen::Matrix4f odom_inited_pose = Eigen::Matrix4f::Identity();
  if (!odom_inited) {
    odom_inited = true;
    odom_inited_pose = curr_reli_odom_.pose * curr_vidar_odom_.pose.inverse();
  }
  curr_vidar_odom_.pose = odom_inited_pose * curr_vidar_odom_.pose;
  return back_end_ptr_->update(curr_cloud_data_, curr_vidar_odom_,
                               curr_reli_odom_);
}

bool BackEndFlow::publishData() {
  transformed_odom_pub_ptr_->publish(curr_vidar_odom_.pose,
                                     curr_vidar_odom_.time);
  if (back_end_ptr_->hasNewKeyFrame()) {
    KeyFrame kf;
    back_end_ptr_->getLatesetKeyFrame(kf);
    key_frame_pub_ptr_->publish(kf);

    back_end_ptr_->getLatesetKeyReliableOdom(kf);
    key_reliable_odom_pub_ptr_->publish(kf);
  }
  if (back_end_ptr_->hasNewOptimized()) {
    std::deque<KeyFrame> optimized_key_frames;
    back_end_ptr_->getOptimizedKFs(optimized_key_frames);
    key_frames_pub_ptr_->publish(optimized_key_frames);
  }
  return true;
}

} // namespace avp_mapping
