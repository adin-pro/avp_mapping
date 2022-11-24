/*
 * @Author: Ren Qian
 * @Date: 2022-10-17 15:33:22
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 19:26:27
 */

#include "tf_listener/tf_listener.hpp"

#include <Eigen/Geometry>

namespace avp_mapping {
TFListener::TFListener(ros::NodeHandle &nh, std::string base_frame_id,
                       std::string child_frame_id)
    : nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {}

bool TFListener::lookUpData(Eigen::Matrix4d &transform_matrix) {
  try {
    tf::StampedTransform transform;
    listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0),
                              transform);
    transformToMatrix(transform, transform_matrix);
    return true;
  } catch (tf::TransformException &ex) {
    return false;
  }
}

bool TFListener::transformToMatrix(const tf::StampedTransform &transform,
                                   Eigen::Matrix4d &transform_matrix) {
  Eigen::Translation3d tl_btol(transform.getOrigin().getX(),
                               transform.getOrigin().getY(),
                               transform.getOrigin().getZ());

  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
  Eigen::AngleAxisd rot_x_btol(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rot_z_btol(yaw, Eigen::Vector3d::UnitZ());

  // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
  transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  return true;
}
} // namespace avp_mapping