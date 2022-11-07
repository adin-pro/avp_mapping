/*
 * @Author: ding.yin
 * @Date: 2022-11-07 15:43:49
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-07 16:02:25
 */

#ifndef _BACK_END_FLOW_H_
#define _BACK_END_FLOW_H_

#include "mapping/back_end/back_end.hpp"

#include <string>

#include "ros/ros.h"

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/loop_pose_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"

#include "publisher/key_frame_publisher.hpp"
#include "publisher/keyframes_publisher.hpp"
#include "publisher/odometry_publisher.hpp"

namespace avp_mapping {
class BackEndFlow {
public:
  BackEndFlow(ros::NodeHandle &nh, const std::string cloud_topic,
              const std::string odom_topic);

  bool run();

  bool forceOptimize();

private:
  bool readData();
  bool hasData();
  bool validData();
  bool publishData();

  bool udpateBackEnd();
  bool maybeInsertLoopPose();

private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> vidar_odom_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> odom_sub_ptr_;
  std::shared_ptr<LoopPoseSubscriber> loop_pose_sub_ptr_;

  std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
  std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
  std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
  std::shared_ptr<BackEnd> back_end_ptr_;

  std::deque<CloudData> cloud_data_buff_;
  std::deque<PoseData> loop_pose_buff_;
  std::deque<PoseData> vidar_odom_buff_;
  std::deque<PoseData> reli_odom_buff_;

  PoseData curr_reli_odom_;
  PoseData curr_vidar_odom_;
  PoseData curr_cloud_data_;
};

} // namespace avp_mapping

#endif // _BACK_END_FLOW_H_