/*
 * @Author: ding.yin
 * @Date: 2022-10-05 20:21:45
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-05 20:24:50
 */
#ifndef _TF_BROADCASTER_H_
#define _TF_BROADCASTER_H_

#include <string>

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

#include "Eigen/Dense"

namespace avp_mapping {
class TFBroadCaster {
public:
  TFBroadCaster() = default;
  TFBroadCaster(std::string frame_id, std::string child_frame_id);
  void sendTransform(Eigen::Matrix4f pose, double time);

protected:
  tf::StampedTransform transform_;
  tf::TransformBroadcaster broadcaster_;
};

} // namespace avp_mapping

#endif // _TF_BROADCASTER_H_