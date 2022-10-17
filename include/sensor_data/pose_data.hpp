/*
 * @Author: Ren Qian
 * @Date: 2020-02-27 23:10:56
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:35:18
 */

#ifndef _POSE_DATA_H_
#define _POSE_DATA_H_

#include <Eigen/Dense>

namespace avp_mapping {
class PoseData {
public:
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double time = 0.0;

public:
  Eigen::Quaternionf getQuaternion();
};
} // namespace avp_mapping

#endif // _POSE_DATA_H_
