/*
 * @Author: ding.yin
 * @Date: 2022-11-09 19:42:09
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-24 21:35:00
 */

#ifndef _LOOP_CLOSING_FLOW_H_
#define _LOOP_CLOSING_FLOW_H_

#include "mapping/loop_close/semantic_loop_closing.hpp"

#include "ros/ros.h"

#include "publisher/loop_pose_publisher.hpp"

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/image_subscriber.hpp"
#include "subscriber/key_frame_subscriber.hpp"

namespace avp_mapping {
class LoopClosingFlow {
public:
  LoopClosingFlow(ros::NodeHandle &nh, std::string work_dir);

  bool run();

private:
  bool readData();
  bool hasData();
  bool validData();
  bool publishData();

private:
  // subscriber
  std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
  std::shared_ptr<CloudSubscriber> kf_cloud_height_sub_ptr_;
  std::shared_ptr<ImageSubscriber> kf_image_sub_ptr_;

  std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;

  std::shared_ptr<SemanticLoopClosing> loop_closing_ptr_;

  std::deque<KeyFrame> key_frame_buff_;
  std::deque<CloudData> cloud_data_buff_;
  std::deque<ImageData> image_data_buff_;

  KeyFrame curr_key_frame_;
  CloudData curr_cloud_height_data_;
  ImageData curr_image_data_;
  LoopPose curr_loop_pose_;

  bool save_loop_pose_ = false;
  std::string save_dir_ = "";
  std::ofstream loop_pose_fd_;
};

} // namespace avp_mapping

#endif // _LOOP_CLOSING_FLOW_H_