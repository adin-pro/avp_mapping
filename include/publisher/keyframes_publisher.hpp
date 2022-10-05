/*
 * @Author: ding.yin
 * @Date: 2022-10-05 19:41:03
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-05 19:43:56
 */
#ifndef _KEYFRAMES_PUBLISHER_H_
#define _KEYFRAMES_PUBLISHER_H_

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <sensor_data/pose_data.hpp>

namespace avp_mapping {
class KeyFramesPublisher {

  KeyFramesPublisher() = default;
  KeyFramesPublisher(ros::NodeHandle &nh, std::string topic_name,
                     std::string frame_id, size_t buff_size);

  void publish(PoseData &pose_data);

  bool hasSubscriber();

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};

} // namespace avp_mapping

#endif // _KEYFRAMES_PUBLISHER_H_