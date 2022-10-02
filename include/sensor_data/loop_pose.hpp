/*
 * @Author: ding.yin
 * @Date: 2022-10-02 20:23:11
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-02 21:19:42
 */

/*
 * @Description: 关键帧之间的相对位姿，用于闭环检测
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 */

#ifndef _LOOP_POSE_H_
#define _LOOP_POSE_H_

#include <Eigen/Dense>

namespace avp_mapping {
class LoopPose {
public:
  double time = 0.0;
  unsigned int index0 = 0;
  unsigned int index1 = 0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

public:
  Eigen::Quaternionf getQuaternion();
};
} // namespace avp_mapping

#endif // _LOOP_POSE_H_