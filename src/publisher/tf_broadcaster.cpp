/*
 * @Author: ding.yin
 * @Date: 2022-10-15 10:23:30
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-15 10:28:12
 */

#include "publisher/tf_broadcaster.hpp"

namespace avp_mapping {
TFBroadCaster::TFBroadCaster(std::string frame_id, std::string child_frame_id) {
  transform_.frame_id_ = frame_id;
  transform_.child_frame_id_ = child_frame_id;
}

void TFBroadCaster::sendTransform(Eigen::Matrix4f pose, double time) {
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
  ros::Time ros_time(static_cast<float>(time));
  transform_.stamp_ = ros_time;
  transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
  broadcaster_.sendTransform(transform_);
}

} // namespace avp_mapping
