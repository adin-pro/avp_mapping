/*
 * @Author: ding.yin
 * @Date: 2022-11-20 20:29:17
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-20 22:22:36
 */

#ifndef _SEMANTICNODE_H_
#define _SEMANTICNODE_H_

#include "global_defination/avp_labels.hpp"
#include <iostream>
#include <vector>

namespace avp_mapping {

struct SemanticNode {
public:
  SemanticNode() = default;
  const std::string getTypeStr();

public:
  int id_ = -1;
  int corr_id_ = -1;
  void printInfo();
  double px_ = 0.0;
  double py_ = 0.0;
  double width_ = 0.0;
  double height_ = 0.0;
  double area_ = 0.0;
  AVPLabels type_ = AVPLabels::BACKGROUND;
  std::vector<double> dist_to_other_nodes_ =
      std::vector<double>(AVPColors.size(), 0);
  std::vector<double> num_other_nodes_ = std::vector<double>(AVPColors.size(), 0);
};

} // namespace avp_mapping

#endif // _SEMANTICNODE_H_