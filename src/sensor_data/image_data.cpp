/*
 * @Author: ding.yin
 * @Date: 2022-10-16 18:58:06
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-12 21:33:20
 */

#include "sensor_data/image_data.hpp"
#include "glog/logging.h"

namespace avp_mapping {

bool ImageData::syncData(std::deque<ImageData> &unsynced_data,
                         double sync_time) {
  while (unsynced_data.size() > 0) {
    if ((sync_time - unsynced_data.front().time) > 0.03) {
      unsynced_data.pop_front();
    } else {
      break;
    }
  }
  if (unsynced_data.size() == 0)
    return false;
  return true;
}

bool ImageData::controlDuration(std::deque<ImageData> &image_deque, double duration) {
  if (image_deque.size() < 2) {
    return false;
  }
  while (image_deque.back().time - image_deque.front().time > duration) {
    image_deque.pop_front();
  }
  return true;
}

bool ImageData::getImageDataByTS(std::deque<ImageData> &data_deque, double timestamp,
                      ImageData &result) {
  int data_size = data_deque.size();
  if (data_size == 0) {
    LOG(WARNING) << "No data in the deque";
    return false;
  }
  if (data_size == 1) {
    double ts_diff = timestamp - data_deque.back().time;
    // timestamp diff should be lower 1/freq 30Hz
    if (ts_diff > 0.04) {
      LOG(WARNING) << "Only one data in the deque, timestamp diff is too large "
                   << timestamp << " " << data_deque.back().time;
      return false;
    }
    result = data_deque.back();
  } else if (timestamp >= data_deque.back().time) {
    double ts_diff = timestamp - data_deque.back().time;
    // timestamp diff should be lower 1/freq 30Hz
    if (ts_diff > 0.04) {
      LOG(WARNING) << "Too large time diff " << timestamp << " "
                   << data_deque.back().time;
      return false;
    }
    result = data_deque.back();
  } else if (timestamp <= data_deque.front().time) {
    double ts_diff = data_deque.front().time - timestamp;
    // timestamp diff should be lower 1/freq 30Hz
    if (ts_diff > 0.04) {
      LOG(WARNING) << "Too large time diff " << timestamp << " "
                   << data_deque.front().time;
      return false;
    }
    result = data_deque.front();
  } else {
    // iteration
    int first_index = -1;
    int second_index = -1;
    for (size_t i = 0; i < data_deque.size() - 1; ++i) {
      if (data_deque[i].time > timestamp) {
        continue;
      } else {
        // data_deque[i].time <= timestamp
        if (data_deque[i + 1].time >= timestamp) {
          first_index = i;
          second_index = i + 1;
          break;
        }
      }
    }
    if (first_index == -1) {
      LOG(ERROR) << "Can not find right timestamp! Deque front_ts: "
                 << data_deque.front().time << " back_ts "
                 << data_deque.back().time << " timestamp " << timestamp;
    }
    result = data_deque[second_index];
  }

  return true;
}

} // namespace avp_mapping