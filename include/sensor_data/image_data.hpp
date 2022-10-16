/*
 * @Author: ding.yin
 * @Date: 2022-10-16 16:23:11
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-16 16:29:39
 */
#ifndef _IMAGE_DATA_H_
#define _IMAGE_DATA_H_

#include "opencv2/opencv.hpp"

namespace {

struct ImageData {
public:
  double time = 0.0;
  cv::Mat image;
};

} // namespace

#endif // _IMAGE_DATA_H_