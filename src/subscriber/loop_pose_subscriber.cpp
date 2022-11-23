/*
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 19:22:00
 */

#include "subscriber/loop_pose_subscriber.hpp"

namespace avp_mapping {

LoopPoseSubscriber::LoopPoseSubscriber(ros::NodeHandle &nh,
                                       std::string topic_name, size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size,
                              &LoopPoseSubscriber::msg_callback, this);
}

void LoopPoseSubscriber::msg_callback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &loop_pose_msg_ptr) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  LoopPose loop_pose;

  loop_pose.time = loop_pose_msg_ptr->header.stamp.toSec();
  loop_pose.index0 =
      static_cast<unsigned int>(loop_pose_msg_ptr->pose.covariance[0]);
  loop_pose.index1 =
      static_cast<unsigned int>(loop_pose_msg_ptr->pose.covariance[1]);
  loop_pose.pose(0, 3) = loop_pose_msg_ptr->pose.pose.position.x;
  loop_pose.pose(1, 3) = loop_pose_msg_ptr->pose.pose.position.y;
  loop_pose.pose(2, 3) = loop_pose_msg_ptr->pose.pose.position.z;

  Eigen::Quaterniond q;
  q.x() = loop_pose_msg_ptr->pose.pose.orientation.x;
  q.y() = loop_pose_msg_ptr->pose.pose.orientation.y;
  q.z() = loop_pose_msg_ptr->pose.pose.orientation.z;
  q.w() = loop_pose_msg_ptr->pose.pose.orientation.w;

  loop_pose.pose.block<3, 3>(0, 0) = q.matrix();
  latest_loop_pose_.push_back(loop_pose);
}

void LoopPoseSubscriber::parseData(std::deque<LoopPose> &looppose_buff) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  if (latest_loop_pose_.size() > 0) {
    looppose_buff.insert(looppose_buff.end(), latest_loop_pose_.begin(),
                         latest_loop_pose_.end());
    latest_loop_pose_.clear();
  }
}

} // namespace avp_mapping