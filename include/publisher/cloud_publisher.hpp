/*
 * @Author: ding.yin
 * @Date: 2022-10-05 18:50:18
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-10 15:26:40
 */
#ifndef _CLOUD_PUBLISHER_H_
#define _CLOUD_PUBLISHER_H_

#include <string>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "sensor_data/cloud_data.hpp"

namespace avp_mapping {
class CloudPublisher {
public:
  CloudPublisher() = default;
  CloudPublisher(ros::NodeHandle &nh, std::string topic_name,
                 std::string frame_id, size_t buff_size);

  void publish(CloudData::CLOUD_PTR &cloud_data_input, double time);
  void publish(CloudData::CLOUD_PTR &cloud_data_input);

  bool hasSubscribers();

private:
  void publishData(CloudData::CLOUD_PTR &cloud_ptr_inputm, ros::Time time);

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};

} // namespace avp_mapping

#endif // _CLOUD_PUBLISHER_H_