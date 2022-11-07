/*
 * @Author: ding.yin 
 * @Date: 2022-11-04 15:16:14 
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-05 20:17:05
 */
#ifndef _FRONT_END_H_
#define _FRONT_END_H_

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <string>

#include "sensor_data/cloud_data.hpp"
#include "sensor_data/key_frame.hpp"
#include "models/registration/registration_interface.hpp"
#include "models/cloud_filter/cloud_filter_interface.hpp"

namespace avp_mapping {

class FrontEnd {
public:
FrontEnd(float key_frame_dist, int local_frame_num);
FrontEnd() = default;

struct Frame{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  CloudData cloud_data;
};

bool setInitPose(const Eigen::Matrix4f init_pose);
bool update(const CloudData& cloud_data, Eigen::Matrix4f& updated_pose);


private:

bool initRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const std::string& type);
bool initFilter(std::shared_ptr<CloudFilterInterface>& filter_ptr, const std::string& type, float leaf_size);
bool udpateWithNewKeyFrame(const Frame& new_key_frame);

std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
std::shared_ptr<RegistrationInterface> registration_ptr_;
std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;

std::deque<Frame> local_map_frames_;

CloudData::CLOUD_PTR local_map_ptr_;
Frame current_frame_;

Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

float key_frame_distance_ = 2.0;
int local_frame_num_ = 20;

};


}



#endif // _FRONT_END_H_