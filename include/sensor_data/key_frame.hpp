/*
 * @Author: ding.yin
 * @Date: 2022-10-02 20:23:36
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-02 21:19:50
 */
/*
 * @Description: 关键帧，在各个模块之间传递数据
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 */

#ifndef _KEY_FRAME_H_
#define _KEY_FRAME_H_

#include <Eigen/Dense>

namespace avp_mapping {
class KeyFrame {
public:
  double time = 0.0;
  unsigned int index = 0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

public:
  Eigen::Quaternionf getQuaternion();
};
} // namespace avp_mapping

#endif // _KEY_FRAME_H_
