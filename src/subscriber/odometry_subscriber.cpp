/*
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 19:22:18
 */

#include "subscriber/odometry_subscriber.hpp"

namespace avp_mapping {

OdometrySubscriber::OdometrySubscriber(ros::NodeHandle &nh,
                                       std::string topic_name, size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size,
                              &OdometrySubscriber::msg_callback, this);
}

void OdometrySubscriber::msg_callback(
    const nav_msgs::OdometryConstPtr &odom_msg_ptr) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  PoseData pose_data;
  pose_data.time = odom_msg_ptr->header.stamp.toSec();
  pose_data.pose(0, 3) = odom_msg_ptr->pose.pose.position.x;
  pose_data.pose(1, 3) = odom_msg_ptr->pose.pose.position.y;
  pose_data.pose(2, 3) = odom_msg_ptr->pose.pose.position.z;

  Eigen::Quaterniond q;
  q.x() = odom_msg_ptr->pose.pose.orientation.x;
  q.y() = odom_msg_ptr->pose.pose.orientation.y;
  q.z() = odom_msg_ptr->pose.pose.orientation.z;
  q.w() = odom_msg_ptr->pose.pose.orientation.w;

  pose_data.pose.block<3, 3>(0, 0) = q.matrix();
  latest_deque_pose_data_.push_back(pose_data);
}

void OdometrySubscriber::parseData(std::deque<PoseData> &odom_buff) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  if (latest_deque_pose_data_.size() > 0) {
    odom_buff.insert(odom_buff.end(), latest_deque_pose_data_.begin(),
                     latest_deque_pose_data_.end());
    latest_deque_pose_data_.clear();
  }
}

} // namespace avp_mapping