/*
 * @Author: ding.yin 
 * @Date: 2022-10-08 20:10:23 
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-08 20:12:09
 */

#include "sensor_data/key_frame.hpp"

namespace avp_mapping {
  Eigen::Quaternionf KeyFrame::getQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);
    return q;
  }
}