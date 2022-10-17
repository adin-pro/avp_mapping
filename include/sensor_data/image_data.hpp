/*
 * @Author: ding.yin
 * @Date: 2022-10-16 16:23:11
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:34:41
 */
#ifndef _IMAGE_DATA_H_
#define _IMAGE_DATA_H_

#include "opencv2/opencv.hpp"

#include <deque>

namespace avp_mapping {

class ImageData {
public:
  double time = 0.0;
  cv::Mat image;
  static bool syncData(std::deque<ImageData> &unsynced_data, double sync_time);
};

} // namespace avp_mapping

#endif // _IMAGE_DATA_H_