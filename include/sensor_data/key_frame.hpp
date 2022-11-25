/*
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:34:53
 */

#ifndef _KEY_FRAME_H_
#define _KEY_FRAME_H_

#include <Eigen/Dense>
#include <deque>

namespace avp_mapping {
class KeyFrame {
public:
  double time = 0.0;
  unsigned int index = 0;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

public:
  Eigen::Quaterniond getQuaternion();
  static bool getKFDataByTS(std::deque<KeyFrame> &data_deque, double timestamp,
                      KeyFrame &result);
};
} // namespace avp_mapping

#endif // _KEY_FRAME_H_
