/*
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:17:00
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 19:09:20
 */

#include "sensor_data/key_frame.hpp"

namespace avp_mapping {
  Eigen::Quaterniond KeyFrame::getQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);
    return q;
  }
}