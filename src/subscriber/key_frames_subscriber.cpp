/*
 * @Author: ding.yin
 * @Date: 2022-10-10 10:50:00
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-10 11:01:25
 */

#include "subscriber/key_frames_subscriber.hpp"

namespace avp_mapping {

KeyFramesSubscriber::KeyFramesSubscriber(ros::NodeHandle &nh,
                                         std::string topic_name,
                                         size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size,
                              &KeyFramesSubscriber::msg_callback, this);
}

void KeyFramesSubscriber::msg_callback(
    const nav_msgs::Path::ConstPtr &key_frames_msg_ptr) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  latest_deque_key_frames_.clear();

  for (size_t i = 0; i < key_frames_msg_ptr->poses.size(); ++i) {
    KeyFrame kf;
    kf.time = key_frames_msg_ptr->poses.at(i).header.stamp.toSec();
    kf.index = static_cast<unsigned int>(i);

    kf.pose(0, 3) = key_frames_msg_ptr->poses.at(i).pose.position.x;
    kf.pose(1, 3) = key_frames_msg_ptr->poses.at(i).pose.position.y;
    kf.pose(2, 3) = key_frames_msg_ptr->poses.at(i).pose.position.z;
    Eigen::Quaternionf q;
    q.x() = key_frames_msg_ptr->poses.at(i).pose.orientation.x;
    q.y() = key_frames_msg_ptr->poses.at(i).pose.orientation.x;
    q.z() = key_frames_msg_ptr->poses.at(i).pose.orientation.x;
    q.w() = key_frames_msg_ptr->poses.at(i).pose.orientation.x;

    kf.pose.block<3, 3>(0, 0) = q.matrix();
    latest_deque_key_frames_.push_back(kf);
  }
}

void KeyFramesSubscriber::parseData(std::deque<KeyFrame> &keyframes_buff) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  if (latest_deque_key_frames_.size() > 0) {
    // keyframes_buff.insert(keyframes_buff.end(),
    // latest_deque_key_frames_.begin(), latest_deque_key_frames_.end());
    keyframes_buff = latest_deque_key_frames_;
    latest_deque_key_frames_.clear();
  }
}

} // namespace avp_mapping
