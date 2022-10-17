/*
 * @Author: ding.yin
 * @Date: 2022-10-16 18:58:06
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-16 22:21:47
 */

#include "sensor_data/image_data.hpp"

namespace avp_mapping {

bool ImageData::syncData(std::deque<ImageData> &unsynced_data,
                         double sync_time) {
  while (unsynced_data.size() > 0) {
    if (unsynced_data.front().time < sync_time) {
      unsynced_data.pop_front();
    } else {
      break;
    }
  }
  if (unsynced_data.size() == 0)
    return false;
  return true;
}

} // namespace avp_mapping