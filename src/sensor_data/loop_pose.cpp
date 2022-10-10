/*
 * @Author: ding.yin 
 * @Date: 2022-10-08 20:13:07 
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-08 21:02:29
 */

#include "sensor_data/loop_pose.hpp"

namespace avp_mapping {
  Eigen::Quaternionf LoopPose::getQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);
    return q;
  }


}