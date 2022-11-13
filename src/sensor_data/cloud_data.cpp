/*
 * @Author: ding.yin
 * @Date: 2022-11-12 17:11:09
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-12 17:46:39
 */

#include "sensor_data/cloud_data.hpp"

namespace avp_mapping {
bool CloudData::controlDuration(std::deque<CloudData> &cloud_deque,
                                double duration) {
  if (cloud_deque.size() < 2) {
    return false;
  }
  while (cloud_deque.back().time - cloud_deque.front().time > duration) {
    cloud_deque.pop_front();
  }
  return true;
}

} // namespace avp_mapping
