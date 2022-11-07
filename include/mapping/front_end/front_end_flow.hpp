/*
 * @Author: ding.yin
 * @Date: 2022-11-05 20:42:52
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-05 21:19:25
 */

#include "ros/ros.h"

#include "mapping/front_end/front_end.hpp"
#include "publisher/odometry_publisher.hpp"
#include "subscriber/cloud_subscriber.hpp"

namespace avp_mapping {
class FrontEndFlow {
public:
  FrontEndFlow(ros::NodeHandle &nh, const std::string &cloud_topic,
               const std::string &odom_topic);
  bool run();

private:
  bool readData();
  bool hasData();
  bool validData();
  bool updateLaserOdometry();
  bool publishData();

private:

  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometryPublisher> odom_pub_ptr_;
  std::shared_ptr<FrontEnd> front_end_ptr_;

  std::deque<CloudData> cloud_data_buffer_;
  CloudData current_cloud_data_;
  Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};

} // namespace avp_mapping
