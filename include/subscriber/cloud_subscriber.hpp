/*
 * @Author: ding.yin
 * @Date: 2022-10-02 21:25:51
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-02 21:34:18
 */

#ifndef _CLOUD_SUBSCRIBER_H_
#define _CLOUD_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <deque>
#include <mutex>
#include <string>

#include "sensor_data/cloud_data.hpp"

namespace avp_mapping {

class CloudSubscriber {
public:
  CloudSubscriber(ros::NodeHandle &nh, std::string topic_name,
                  size_t buffer_size);
  CloudSubscriber() = default;
  void parseData(std::deque<CloudData> &deque_cloud_data);

private:
  void msg_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr);

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<CloudData> latest_deque_cloud_data_;

  std::mutex buff_mutex_;
};

} // namespace avp_mapping

#endif // _CLOUD_SUBSCRIBER_H_
