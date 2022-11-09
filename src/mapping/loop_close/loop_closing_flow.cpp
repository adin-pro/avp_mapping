/*
 * @Author: ding.yin
 * @Date: 2022-11-09 20:37:37
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-09 20:47:35
 */

#include "mapping/loop_close/loop_closing_flow.hpp"

namespace avp_mapping {
LoopClosingFlow::LoopClosingFlow(ros::NodeHandle &nh, std::string work_dir) {
  key_frame_sub_ptr_ =
      std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 1000);
  reli_odom_sub_ptr_ =
      std::make_shared<KeyFrameSubscriber>(nh, "/key_reliable_odom", 1000);
  loop_pose_pub_ptr_ =
      std::make_shared<LoopPosePublisher>(nh, "/loop_pose", "/base_link", 100);
  loop_closing_ptr_ = std::make_shared<LoopClosing>(work_dir);
}

bool LoopClosingFlow::run() {
  if (!readData())
    return false;
  while (hasData()) {
    if (validData()) {
      continue;
    }
    loop_closing_ptr_->update(curr_key_frame_, curr_reli_odom_);
    publishData();
  }
  return true;
}

bool LoopClosingFlow::readData() {
  key_frame_sub_ptr_->parseData(key_frame_buff_);
  reli_odom_sub_ptr_->parseData(reli_odom_buff_);
  return true;
}

bool LoopClosingFlow::hasData() {
  if (key_frame_buff_.size() == 0)
    return false;
  if (reli_odom_buff_.size() == 0)
    return false;
  return true;
}

bool LoopClosingFlow::validData() {
  curr_key_frame_ = key_frame_buff_.front();
  curr_reli_odom_ = reli_odom_buff_.front();
  double time_diff = curr_key_frame_.time - curr_reli_odom_.time;
  if (time_diff < -0.05) {
    key_frame_buff_.pop_front();
    return false;
  }

  if (time_diff > 0.05) {
    reli_odom_buff_.pop_front();
    return false;
  }

  key_frame_buff_.pop_front();
  reli_odom_buff_.pop_front();
  return true;
}

bool LoopClosingFlow::publishData() {
  if (loop_closing_ptr_->hasNewLoopPose()) {
    loop_pose_pub_ptr_->publish(loop_closing_ptr_->getCurrentLoopPose());
  }
  return true;
}

} // namespace avp_mapping