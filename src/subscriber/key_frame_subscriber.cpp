/*
 * @Author: ding.yin
 * @Date: 2022-10-08 19:28:11
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-08 19:39:24
 */
#include "subscriber/key_frame_subscriber.hpp"

namespace avp_mapping {

KeyFrameSubscriber::KeyFrameSubscriber(ros::NodeHandle &nh,
                                       std::string topic_name, size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size,
                              &KeyFrameSubscriber::msg_callback, this);
}

void KeyFrameSubscriber::msg_callback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &key_frame_msg_ptr) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  KeyFrame kf;
  kf.time = key_frame_msg_ptr->header.stamp.toSec();
  kf.index = static_cast<unsigned int>(key_frame_msg_ptr->pose.covariance[0]);

  kf.pose(0, 3) = key_frame_msg_ptr->pose.pose.position.x;
  kf.pose(1, 3) = key_frame_msg_ptr->pose.pose.position.y;
  kf.pose(2, 3) = key_frame_msg_ptr->pose.pose.position.z;

  Eigen::Quaternionf q;
  q.x() = key_frame_msg_ptr->pose.pose.orientation.x;
  q.y() = key_frame_msg_ptr->pose.pose.orientation.y;
  q.z() = key_frame_msg_ptr->pose.pose.orientation.z;
  q.w() = key_frame_msg_ptr->pose.pose.orientation.w;
  kf.pose.block<3, 3>(0, 0) = q.matrix();
  latest_deque_key_frame_.push_back(kf);
}

void KeyFrameSubscriber::parseData(std::deque<KeyFrame> &key_frame_buff) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  if (latest_deque_key_frame_.size() > 0) {
    key_frame_buff.insert(key_frame_buff.end(), latest_deque_key_frame_.begin(),
                          latest_deque_key_frame_.end());
    latest_deque_key_frame_.clear();
  }
}

} // namespace avp_mapping