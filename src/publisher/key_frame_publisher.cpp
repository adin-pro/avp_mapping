/*
 * @Author: ding.yin
 * @Date: 2022-10-10 16:16:56
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-10 16:26:18
 */

#include "publisher/key_frame_publisher.hpp"

namespace avp_mapping {

KeyFramePublisher::KeyFramePublisher(ros::NodeHandle &nh,
                                     std::string topic_name,
                                     std::string frame_id, size_t buff_size)
    : nh_(nh), frame_id_(frame_id) {
  publisher_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      topic_name, buff_size);
}

void KeyFramePublisher::publish(KeyFrame &kf) {
  geometry_msgs::PoseWithCovarianceStamped pose_stamped;
  ros::Time ros_time(static_cast<float>(kf.time));
  pose_stamped.header.stamp = ros_time;
  pose_stamped.header.frame_id = frame_id_;

  pose_stamped.pose.pose.position.x = kf.pose(0, 3);
  pose_stamped.pose.pose.position.y = kf.pose(1, 3);
  pose_stamped.pose.pose.position.z = kf.pose(2, 3);

  Eigen::Quaternionf q = kf.getQuaternion();
  pose_stamped.pose.pose.orientation.x = q.x();
  pose_stamped.pose.pose.orientation.y = q.y();
  pose_stamped.pose.pose.orientation.z = q.z();
  pose_stamped.pose.pose.orientation.w = q.w();

  pose_stamped.pose.covariance[0] = static_cast<double>(kf.index);
  publisher_.publish(pose_stamped);
}

bool KeyFramePublisher::hasSubscriber() {
  return publisher_.getNumSubscribers() != 0;
}

} // namespace avp_mapping
