/*
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:39:28
 */
#ifndef _LOOP_POSE_PUBLISHER_H_
#define _LOOP_POSE_PUBLISHER_H_

#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include "sensor_data/loop_pose.hpp"

namespace avp_mapping {

class LoopPosePublisher {
public:
  LoopPosePublisher() = default;
  LoopPosePublisher(ros::NodeHandle &nh_, std::string topic_name,
                    std::string frame_id, size_t buff_size);

  void publish(LoopPose &loop_pose);

  bool hasSubscriber();

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};

} // namespace avp_mapping

#endif // _LOOP_POSE_PUBLISHER_H_