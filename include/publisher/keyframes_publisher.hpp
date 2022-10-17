/*
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:39:09
 */
#ifndef _KEYFRAMES_PUBLISHER_H_
#define _KEYFRAMES_PUBLISHER_H_

#include <string>
#include <deque>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include "nav_msgs/Path.h"
#include "sensor_data/key_frame.hpp"

namespace avp_mapping {
class KeyFramesPublisher {

  KeyFramesPublisher() = default;
  KeyFramesPublisher(ros::NodeHandle &nh, std::string topic_name,
                     std::string frame_id, size_t buff_size);

  void publish(const std::deque<KeyFrame>& kfs);

  bool hasSubscriber();

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};

} // namespace avp_mapping

#endif // _KEYFRAMES_PUBLISHER_H_