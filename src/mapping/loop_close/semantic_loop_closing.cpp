/*
 * @Author: ding.yin
 * @Date: 2022-11-24 15:33:46
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-24 22:19:54
 */

#include "mapping/loop_close/semantic_loop_closing.hpp"

#include <algorithm>
#include <numeric>

namespace avp_mapping {

template <typename T> std::vector<int> argsort(const std::vector<T> &array);

SemanticLoopClosing::SemanticLoopClosing(YAML::Node &node) {
  // loop pose should be far away from each other
  loop_pose_step_ = node["loop_pose_step"].as<int>();
  // avoid mismatch
  start_up_index_ = node["start_up_index"].as<int>();
  // avoid mismatch
  index_exclude_range_ = node["index_exclude_range"].as<int>();
  // prior information for semantic objects
  prior_size_ = node["prior_size"].as<std::vector<double>>();
  // min limit for coarse similarity
  coarse_similarity_limit_ = node["coarse_similarity_limit"].as<double>();
  // min limit for fine similarity
  fine_similarity_limit_ = node["fine_similarity_limit"].as<double>();

  // icp
  max_iteration_ = node["max_iteration"].as<int>();
  fitness_upper_bound_ = node["fitness_upper_bound"].as<double>();
  icp.setMaximumIterations(max_iteration_);

  // coeffs for calculating similarity, different semantic labels have different
  // weights
  coarse_semantic_coeff_ =
      node["coarse_semantic_coeff"].as<std::vector<double>>();
  fine_semantic_coeff_ = node["fine_semantic_coeff"].as<std::vector<double>>();

  coarse_candidates_ = node["coarse_candidates"].as<int>();

  SemanticList::coarse_semantic_coeff_ = coarse_semantic_coeff_;
  SemanticList::find_semantic_coeff_ = fine_semantic_coeff_;
}

bool SemanticLoopClosing::TryGetLoopPose(const ImageData &image_data,
                                         CloudData &cloud_height, LoopPose &lp,
                                         const KeyFrame &kf) {

  SemanticList queryList = CreateSListFromImage(image_data, kf);
  InsertSList(queryList, cloud_height);
  if (queryList.id_ < start_up_index_) {
    return false;
  }

  if (queryList.id_ - last_loop_pose_index < loop_pose_step_) {
    return false;
  }

  if (!HasEnoughObjects(queryList))
    return false;
  SemanticList answerList;
  if (!FindBestMatch(queryList, answerList))
    return false;
  Eigen::Matrix4d qurey_to_answer;
  if (!CalcRelativePoseToTarget(queryList, answerList, qurey_to_answer))
    return false;
  last_loop_pose_index = queryList.id_;
  lp.index0 = queryList.id_;
  lp.index1 = answerList.id_;
  lp.time = image_data.time;
  lp.pose = qurey_to_answer;
  return true;
}

SemanticList
SemanticLoopClosing::CreateSListFromImage(const ImageData &image_data,
                                          const KeyFrame &kf) {
  cv::Mat img = image_data.image;
  cv::Mat crossing;
  cv::Mat lane_line;
  cv::Mat parking_line;
  cv::Mat marker;
  cv::Mat stop_line;
  cv::Mat bumper;
  cv::inRange(img,
              cv::Scalar(AVPColors[AVPLabels::CROSSING].z(),
                         AVPColors[AVPLabels::CROSSING].y(),
                         AVPColors[AVPLabels::CROSSING].x()),
              cv::Scalar(AVPColors[AVPLabels::CROSSING].z(),
                         AVPColors[AVPLabels::CROSSING].y(),
                         AVPColors[AVPLabels::CROSSING].x()),
              crossing);

  cv::inRange(img,
              cv::Scalar(AVPColors[AVPLabels::LANE_LINE].z(),
                         AVPColors[AVPLabels::LANE_LINE].y(),
                         AVPColors[AVPLabels::LANE_LINE].x()),
              cv::Scalar(AVPColors[AVPLabels::LANE_LINE].z(),
                         AVPColors[AVPLabels::LANE_LINE].y(),
                         AVPColors[AVPLabels::LANE_LINE].x()),
              lane_line);

  cv::inRange(img,
              cv::Scalar(AVPColors[AVPLabels::PARKING_LINE].z(),
                         AVPColors[AVPLabels::PARKING_LINE].y(),
                         AVPColors[AVPLabels::PARKING_LINE].x()),
              cv::Scalar(AVPColors[AVPLabels::PARKING_LINE].z(),
                         AVPColors[AVPLabels::PARKING_LINE].y(),
                         AVPColors[AVPLabels::PARKING_LINE].x()),
              parking_line);
  cv::inRange(img,
              cv::Scalar(AVPColors[AVPLabels::MARKER].z(),
                         AVPColors[AVPLabels::MARKER].y(),
                         AVPColors[AVPLabels::MARKER].x()),
              cv::Scalar(AVPColors[AVPLabels::MARKER].z(),
                         AVPColors[AVPLabels::MARKER].y(),
                         AVPColors[AVPLabels::MARKER].x()),
              marker);
  cv::inRange(img,
              cv::Scalar(AVPColors[AVPLabels::STOP_LINE].z(),
                         AVPColors[AVPLabels::STOP_LINE].y(),
                         AVPColors[AVPLabels::STOP_LINE].x()),
              cv::Scalar(AVPColors[AVPLabels::STOP_LINE].z(),
                         AVPColors[AVPLabels::STOP_LINE].y(),
                         AVPColors[AVPLabels::STOP_LINE].x()),
              stop_line);
  cv::inRange(
      img,
      cv::Scalar(AVPColors[AVPLabels::BUMP].z(), AVPColors[AVPLabels::BUMP].y(),
                 AVPColors[AVPLabels::BUMP].x()),
      cv::Scalar(AVPColors[AVPLabels::BUMP].z(), AVPColors[AVPLabels::BUMP].y(),
                 AVPColors[AVPLabels::BUMP].x()),
      bumper);

  std::vector<std::vector<double>> crossing_rot_vec;
  std::vector<std::vector<double>> marker_rot_vec;
  std::vector<std::vector<double>> lane_line_rot_vec;
  std::vector<std::vector<double>> parking_line_rot_vec;
  std::vector<std::vector<double>> stop_line_rot_vec;
  std::vector<std::vector<double>> bumper_rot_vec;

  find_min_rects_crossing(crossing, crossing_rot_vec,
                          prior_size_[AVPLabels::CROSSING]);
  find_min_rects(lane_line, lane_line_rot_vec,
                 prior_size_[AVPLabels::LANE_LINE]);
  find_min_rects(parking_line, parking_line_rot_vec,
                 prior_size_[AVPLabels::PARKING_LINE]);
  find_min_rects(marker, marker_rot_vec, prior_size_[AVPLabels::MARKER]);
  find_min_rects(stop_line, stop_line_rot_vec,
                 prior_size_[AVPLabels::STOP_LINE]);
  find_min_rects(bumper, bumper_rot_vec, prior_size_[AVPLabels::BUMP]);

  SemanticList slist;
  slist.half_height_ = image_data.image.rows / 2;
  slist.half_width_ = image_data.image.cols / 2;
  slist.setId(kf.index);
  slist.addSemanticNode(lane_line_rot_vec, AVPLabels::LANE_LINE);
  slist.addSemanticNode(parking_line_rot_vec, AVPLabels::PARKING_LINE);
  slist.addSemanticNode(marker_rot_vec, AVPLabels::MARKER);
  slist.addSemanticNode(crossing_rot_vec, AVPLabels::CROSSING);
  slist.addSemanticNode(bumper_rot_vec, AVPLabels::BUMP);
  slist.addSemanticNode(stop_line_rot_vec, AVPLabels::STOP_LINE);
  slist.procNodes(); // calc distance
  return slist;
}

bool SemanticLoopClosing::HasEnoughObjects(const SemanticList &sList) {
  return std::accumulate(sList.num_objects_.begin() + 4,
                         sList.num_objects_.end(), 0) > 4;
}

bool SemanticLoopClosing::InsertSList(const SemanticList &slist,
                                      CloudData &cloud_height) {
  sList_database_.emplace_back(slist);
  kf_cloud_height_database_[slist.id_] = cloud_height;
  return true;
}

bool SemanticLoopClosing::FindBestMatch(const SemanticList &query,
                                        SemanticList &answer) {
  if (sList_database_.size() <= 1)
    return false;
  std::vector<double> coarse_similarities;
  // coarse similarity
  for (size_t i = 0; i < sList_database_.size(); ++i) {
    coarse_similarities.push_back(
        SemanticList::coarseSimilarity(query, sList_database_[i]));
  }
  auto ind_arg = argsort(coarse_similarities);
  int candidates_num = 0;
  int best_ind = -1;
  double best_fine_similarity = 0.0;
  // fine similarity
  for (int candidate_ind = 0; candidate_ind < static_cast<int>(coarse_similarities.size()) &&
                              candidates_num < coarse_candidates_;
       ++candidate_ind) {
    if (abs(query.id_ - sList_database_[ind_arg[candidate_ind]].id_) <
        index_exclude_range_)
      continue;
    candidates_num++;
    double fine_similarity = SemanticList::fineSimilarity(
        query, sList_database_[ind_arg[candidate_ind]]);
    if (fine_similarity > best_fine_similarity) {
      best_fine_similarity = fine_similarity;
      best_ind = ind_arg[candidate_ind];
    }
  }

  if (best_ind == -1)
    return false;
  if (coarse_similarities[best_ind] < coarse_similarity_limit_)
    return false;
  if (best_fine_similarity < fine_similarity_limit_)
    return false;
  answer = sList_database_[best_ind];
  return true;
}

bool SemanticLoopClosing::CalcRelativePoseToTarget(
    SemanticList &source, SemanticList &target,
    Eigen::Matrix4d &trans_src_to_tar) {
  LOG(INFO) << "KF " << source.id_ << " & KF " << target.id_;
  Eigen::Matrix4d guess_pose = SemanticList::getTransformation(source, target);
  icp.setInputSource(kf_cloud_height_database_[source.id_].cloud_ptr);
  icp.setInputTarget(kf_cloud_height_database_[target.id_].cloud_ptr);
  icp.align(*kf_cloud_height_database_[source.id_].cloud_ptr,
            guess_pose.cast<float>());
  if (icp.hasConverged() && icp.getFitnessScore() < fitness_upper_bound_) {
    trans_src_to_tar = icp.getFinalTransformation().cast<double>();
    if (fabs(trans_src_to_tar(2, 3)) > 0.1)
      return false;
    LOG(INFO) << "Success ICP fitness " << icp.getFitnessScore();
  } else {
    LOG(INFO) << "Failed ICP fitness " << icp.getFitnessScore();
    return false;
  }

  return true;
}

template <typename T> std::vector<int> argsort(const std::vector<T> &array) {
  const int array_len(array.size());
  std::vector<int> array_index(array_len, 0);
  for (int i = 0; i < array_len; ++i) {
    array_index[i] = i;
  }
  std::sort(array_index.begin(), array_index.end(),
            [&array](int pos1, int pos2) { return array[pos1] > array[pos2]; });
  return array_index;
}

void SemanticLoopClosing::find_min_rects(
    cv::Mat &img, std::vector<std::vector<double>> &out_rects,
    double min_area) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE);
  for (auto &vec : contours) {
    cv::RotatedRect rot_rect = cv::minAreaRect(vec);
    if (rot_rect.size.area() < min_area)
      continue;
    out_rects.push_back(std::vector<double>{
        rot_rect.center.x, rot_rect.center.y, rot_rect.size.width,
        rot_rect.size.height, rot_rect.size.area()});
    cv::Point2f p[4];
    rot_rect.points(p);
  }
}

void SemanticLoopClosing::find_min_rects_crossing(
    cv::Mat img, std::vector<std::vector<double>> &out_rects, double min_area) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE);
  std::unordered_map<double, std::vector<int>> groups;
  std::unordered_map<double, double> hash_x;
  std::unordered_map<double, double> hash_y;

  for (size_t i = 0; i < contours.size(); ++i) {
    cv::RotatedRect rot_rect = cv::minAreaRect(contours[i]);

    if (rot_rect.size.area() < min_area) {
      continue;
    }
    cv::Point2f p[4];
    rot_rect.points(p);
    cv::Point2f mid_p_1(-10.0, -10.0);
    cv::Point2f mid_p_2(-10.0, -10.0);
    for (int i_th = 0; i_th < 4; ++i_th) {
      double len =
          (p[i_th].x - p[(i_th + 1) % 4].x) *
              (p[i_th].x - p[(i_th + 1) % 4].x) +
          (p[i_th].y - p[(i_th + 1) % 4].y) * (p[i_th].y - p[(i_th + 1) % 4].y);
      if (std::sqrt(len) <
          std::max(rot_rect.size.width, rot_rect.size.height)) {
        if (mid_p_1.x < -5.0) {
          mid_p_1.x = (p[i_th].x + p[(i_th + 1) % 4].x) * 0.5;
          mid_p_1.y = (p[i_th].y + p[(i_th + 1) % 4].y) * 0.5;
        } else {
          mid_p_2.x = (p[i_th].x + p[(i_th + 1) % 4].x) * 0.5;
          mid_p_2.y = (p[i_th].y + p[(i_th + 1) % 4].y) * 0.5;
        }
      }
    }
    Eigen::Vector2d center_line_vec(mid_p_1.x - mid_p_2.x,
                                    mid_p_1.y - mid_p_2.y);
    Eigen::Vector2d x_line_vec(1, 0);
    if (center_line_vec.y() < 0) {
      center_line_vec.x() = -center_line_vec.x();
      center_line_vec.y() = -center_line_vec.y();
    }

    double angle = std::acos(x_line_vec.dot(center_line_vec) /
                             (x_line_vec.norm() * center_line_vec.norm()));

    bool insert_flag = false;
    for (auto &p : groups) {
      if ((abs(p.first - angle) < 20.0 / 180.0 * M_PI) ||
          (abs(abs(p.first - angle) - M_PI) < 20.0 / 180.0 * M_PI)) {
        double dist = sqrt((hash_x[p.first] - rot_rect.center.x) *
                               (hash_x[p.first] - rot_rect.center.x) +
                           (hash_y[p.first] - rot_rect.center.y) *
                               (hash_y[p.first] - rot_rect.center.y));
        if (dist > 120.0) {
          continue;
        }
        p.second.push_back(static_cast<int>(i));
        insert_flag = true;
        break;
      }
    }
    if (insert_flag == false) {
      double angle_key = angle + 0.01 * static_cast<double>(rand()) /
                                     static_cast<double>(RAND_MAX);
      groups.insert(std::pair<double, std::vector<int>>(
          angle_key, std::vector<int>{static_cast<int>(i)}));
      hash_x[angle_key] = rot_rect.center.x;
      hash_y[angle_key] = rot_rect.center.y;
    }
  }

  for (auto &p : groups) {

    std::vector<cv::Point> group_contour;
    for (auto &i : p.second) {
      group_contour.insert(group_contour.end(), contours[i].begin(),
                           contours[i].end());
    }
    cv::RotatedRect rot_rect = cv::minAreaRect(group_contour);

    if (rot_rect.size.area() < 1000) {
      continue;
    }
    out_rects.push_back(std::vector<double>{
        rot_rect.center.x, rot_rect.center.y, rot_rect.size.width,
        rot_rect.size.height, rot_rect.size.area()});
  }
}

} // namespace avp_mapping
