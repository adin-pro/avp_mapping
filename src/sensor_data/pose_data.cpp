/*
 * @Author: Ren Qian
 * @Date: 2020-02-28 18:50:16
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 19:09:58
 */

#include "sensor_data/pose_data.hpp"

namespace avp_mapping {

Eigen::Quaternionf PoseData::getQuaternion() {
  Eigen::Quaternionf q;
  q = pose.block<3, 3>(0, 0);
  return q;
}

} // namespace avp_mapping
