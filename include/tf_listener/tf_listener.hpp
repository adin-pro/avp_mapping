/*
 * @Author: Ren Qian
 * @Date: 2020-02-06 16:01:21
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:43:32
 */
#ifndef _TF_LISTENER_H_
#define _TF_LISTENER_H_

#include <string>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <Eigen/Dense>

namespace avp_mapping {

class TFListener {

public:
  TFListener() = default;
  TFListener(ros::NodeHandle &nh, std::string base_frame_id,
             std::string child_frame_id);

  bool lookUpData(Eigen::Matrix4f &transform_matrix);

private:
  bool transformToMatrix(const tf::StampedTransform &transform,
                         Eigen::Matrix4f &transform_matrix);

private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  std::string base_frame_id_;
  std::string child_frame_id_;
};

} // namespace avp_mapping

#endif // _TF_LISTENER_H_