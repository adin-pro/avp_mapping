/*
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:45:32
 */

#include "publisher/keyframes_publisher.hpp"

namespace avp_mapping {

KeyFramesPublisher::KeyFramesPublisher(ros::NodeHandle &nh,
                                       std::string topic_name,
                                       std::string frame_id, size_t buff_size)
    : nh_(nh), frame_id_(frame_id) {
  publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
}

void KeyFramesPublisher::publish(const std::deque<KeyFrame> &kfs) {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = frame_id_;

  for (size_t i = 0; i < kfs.size(); ++i) {
    KeyFrame key_frame = kfs.at(i);
    geometry_msgs::PoseStamped pose_stamped;
    ros::Time ros_time(static_cast<float>(key_frame.time));

    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = frame_id_;
    pose_stamped.header.seq = key_frame.index;

    pose_stamped.pose.position.x = key_frame.pose(0, 3);
    pose_stamped.pose.position.y = key_frame.pose(1, 3);
    pose_stamped.pose.position.z = key_frame.pose(2, 3);

    Eigen::Quaternionf q = key_frame.getQuaternion();
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    path.poses.push_back(pose_stamped);
  }

  publisher_.publish(path);
}

bool KeyFramesPublisher::hasSubscriber() {
  return publisher_.getNumSubscribers() != 0;
}

} // namespace avp_mapping