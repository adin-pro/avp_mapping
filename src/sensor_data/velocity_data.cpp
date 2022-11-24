/*
 * @Author: Ren Qian
 * @Date: 2020-02-23 22:20:41
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 19:10:19
 */
#include "sensor_data/velocity_data.hpp"

namespace avp_mapping {

bool VelocityData::syncData(std::deque<VelocityData> &unsyncedData,
                            std::deque<VelocityData> &syncedData,
                            double sync_time) {
  while (unsyncedData.size() > 2) {
    if (unsyncedData.front().time > sync_time) {
      return false;
    }
    if (unsyncedData.at(1).time < sync_time) {
      unsyncedData.pop_front();
      continue;
    }
    if (sync_time - unsyncedData.front().time > 0.2) {
      unsyncedData.pop_front();
      break;
    }
    if (unsyncedData.at(1).time - sync_time > 0.2) {
      unsyncedData.pop_front();
      break;
    }
  }

  if (unsyncedData.size() < 2)
    return false;

  VelocityData front_data = unsyncedData.front();
  VelocityData back_data = unsyncedData.at(1);
  VelocityData synced_data;

  double front_scale =
      (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale =
      (sync_time - front_data.time) / (back_data.time - front_data.time);

  synced_data.time = sync_time;
  synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale +
                                  back_data.linear_velocity.x * back_scale;
  synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale +
                                  back_data.linear_velocity.y * back_scale;
  synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale +
                                  back_data.linear_velocity.z * back_scale;

  synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale +
                                   back_data.angular_velocity.x * back_scale;
  synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale +
                                   back_data.angular_velocity.y * back_scale;
  synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale +
                                   back_data.angular_velocity.z * back_scale;

  syncedData.push_back(synced_data);
  return true;
}

void VelocityData::transformCoordinate(Eigen::Matrix4d transform_matrix) {
  Eigen::Matrix4d matrix = transform_matrix.cast<double>();
  Eigen::Matrix3d t_R = matrix.block<3, 3>(0, 0);
  Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
  Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);

  w = t_R * w;
  v = t_R * v;

  Eigen::Vector3d r(matrix(0, 3), matrix(1, 3), matrix(2, 3));
  Eigen::Vector3d delta_v;
  delta_v(0) = w(1) * r(2) - w(2) * r(1);
  delta_v(1) = w(2) * r(0) - w(0) * r(2);
  delta_v(2) = w(1) * r(1) - w(1) * r(0);

  v + delta_v;
  angular_velocity.x = w(0);
  angular_velocity.y = w(1);
  angular_velocity.z = w(2);
  linear_velocity.x = v(0);
  linear_velocity.y = v(1);
  linear_velocity.z = v(2);
}

} // namespace avp_mapping