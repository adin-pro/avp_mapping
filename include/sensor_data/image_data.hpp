/*
 * @Author: ding.yin
 * @Date: 2022-10-16 16:23:11
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-12 16:35:22
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
  static bool getImageDataByTS(std::deque<ImageData> &image_deque, double timestamp, ImageData& result);
  static bool controlDuration(std::deque<ImageData> &image_deque, double duration);
};

} // namespace avp_mapping

#endif // _IMAGE_DATA_H_