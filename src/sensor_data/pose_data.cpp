/*
 * @Author: ding.yin
 * @Date: 2022-10-10 10:10:25
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-10 10:13:03
 */

#include "sensor_data/pose_data.hpp"

namespace avp_mapping {

Eigen::Quaternionf PoseData::getQuaternion() {
  Eigen::Quaternionf q;
  q = pose.block<3, 3>(0, 0);
  return q;
}

} // namespace avp_mapping
