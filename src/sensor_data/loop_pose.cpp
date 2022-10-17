/*
 * @Author: Ren Qian 
 * @Date: 2020-02-28 19:17:00
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 19:09:41
 */

#include "sensor_data/loop_pose.hpp"

namespace avp_mapping {
  Eigen::Quaternionf LoopPose::getQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);
    return q;
  }


}