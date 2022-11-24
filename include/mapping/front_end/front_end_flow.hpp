/*
 * @Author: ding.yin
 * @Date: 2022-11-05 20:42:52
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-05 21:19:25
 */

#include "ros/ros.h"

#include "mapping/front_end/front_end.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/image_publisher.hpp"
#include "publisher/key_frame_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/image_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"

namespace avp_mapping {
class FrontEndFlow {
public:
  FrontEndFlow(ros::NodeHandle &nh, const std::string &work_dir);
  bool run();

private:
  bool readData();
  bool hasData();
  bool validData();
  bool updateOdometry();
  bool HasNewKF();
  bool publishOdom();
  bool publishKFData();
  bool saveData();

private:
  // subscribers
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<CloudSubscriber> cloud_height_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> odom_sub_ptr_;
  std::shared_ptr<ImageSubscriber> image_sub_ptr_;
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
  // publisher
  std::shared_ptr<CloudPublisher> kf_cloud_pub_ptr_;
  std::shared_ptr<CloudPublisher> kf_cloud_height_pub_ptr_;
  std::shared_ptr<ImagePublisher> kf_image_pub_ptr_;
  std::shared_ptr<OdometryPublisher> odom_pub_ptr_;
  std::shared_ptr<KeyFramePublisher> kf_pub_ptr_;
  // front end algorithm module
  std::shared_ptr<FrontEnd> front_end_ptr_;
  // data buffer
  std::deque<CloudData> cloud_data_buffer_;
  std::deque<CloudData> cloud_height_data_buffer_;
  std::deque<ImageData> image_data_buffer_;
  std::deque<PoseData> odom_data_buffer_;
  std::deque<IMUData> imu_data_buffer_;
  // current data
  CloudData curr_cloud_data_;
  CloudData curr_cloud_height_data_;
  ImageData curr_image_data_;
  PoseData curr_odom_data_;
  PoseData last_odom_data_;
  IMUData curr_imu_data_;
  // frame poses
  PoseData last_frame_pose_;
  PoseData curr_frame_pose_;
  PoseData last_kf_pose_;
  // add scale
  bool add_scale_ = false;
  double yaw_scale_ = 1.000;

  bool save_kf_ = false;
  int kf_id_ = -1;
  double kf_sync_time_ = 0.0;
  double key_frame_dist_ = 2.0;
  double sub_frame_dist_ = 0.5; // for occupancy map
  double control_duration_ = 10.0;
  std::string kf_cloud_save_path_ = "";
  std::string kf_cloud_height_save_path_ = "";
  std::string kf_image_save_path_ = "";
  std::ofstream kf_pose_fd_;
};

} // namespace avp_mapping
