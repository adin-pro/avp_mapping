/*
 * @Author: ding.yin
 * @Date: 2022-11-07 15:43:49
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-25 16:23:19
 */

#ifndef _BACK_END_FLOW_H_
#define _BACK_END_FLOW_H_

#include "mapping/back_end/pgo_back_end.hpp"

#include <fstream>
#include <string>

#include "ros/ros.h"

#include "sensor_data/pose_data.hpp"

#include "subscriber/key_frame_subscriber.hpp"
#include "subscriber/loop_pose_subscriber.hpp"

#include "publisher/keyframes_publisher.hpp"

namespace avp_mapping {
class BackEndFlow {
public:
  BackEndFlow(ros::NodeHandle &nh, std::string work_dir);

  bool run();

private:
  bool readData();
  bool hasData();
  bool publishData();
  bool maybeInsertLoopPose();
  bool MaybeInsertKFOdometry();

private:
  std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
  std::shared_ptr<LoopPoseSubscriber> loop_pose_sub_ptr_;

  std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
  std::shared_ptr<PGOBackEnd> pgo_back_end_ptr_;

  std::deque<KeyFrame> key_frame_buff_;
  std::deque<LoopPose> loop_pose_buff_;
  std::deque<KeyFrame> optimized_kfs_;

  LoopPose curr_loop_pose_;
  KeyFrame curr_key_frame_;
  KeyFrame last_key_frame_;

  std::vector<double> odom_info_;
  std::vector<double> loop_pose_info_;
};

} // namespace avp_mapping

#endif // _BACK_END_FLOW_H_