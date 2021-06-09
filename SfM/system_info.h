/// \file src/system_info.h
/// \author zxm
/// \email xingmingzhangssr@gmail
/// \version 0.1
/// \brief define some class manipulate global info of SfM system

#ifndef __monocular_system_info_h__
#define __monocular_system_info_h__

#include <corecrt_math_defines.h>

#include <opencv2/core.hpp>
#include <shared_mutex>
#include <vector>

#include "common_type.h"

/// \brief System info
class SystemInfo {
 public:
  // statistical info for system running time
  class SysTimeInfo;

  // statistical info for other infos
  class SysStatistics;

  // system running config
  struct SysConfig;

 private:
  cv::Mat1d _k;                      ///< camera matrix K(fx, fy, cx, cy)
  std::vector<double> _dist_coeffs;  ///< camera disturb
  mutable smtx _mtx;  ///< shared mutex for accessing unusually changed variable

  sp<SysTimeInfo> _time;

  sp<SysStatistics> _record;

  sp<SysConfig> _conf;

 public:
  SystemInfo();
  ~SystemInfo();

  void camera_K(double fx, double fy, double cx, double cy) {
    if (_k.empty()) _k.create(3, 3);
    ulock<smtx> lock(_mtx);
    _k << fx, 0., cx, 0., fy, cy, 0., 0., 1.;
  }

  const cv::Mat& camera_K() const {
    slock<smtx> lock(_mtx);
    return _k;
  }

  std::tuple<double, double, double, double> camera_parameters() const {
    slock<smtx> lock(_mtx);
    double fx = _k.at<double>(0, 0), fy = _k.at<double>(1, 1),
           cx = _k.at<double>(0, 2), cy = _k.at<double>(1, 2);
    return {fx, fy, cx, cy};
  }

  void distort_parameter(double k1, double k2, double p1, double p2) {
    ulock<smtx> lock(_mtx);
    if (_dist_coeffs.empty()) _dist_coeffs.resize(4, 0.);
    _dist_coeffs[0] = k1;
    _dist_coeffs[1] = k2;
    _dist_coeffs[2] = p1;
    _dist_coeffs[3] = p2;
  }

  const std::vector<double>& distort_parameter() const {
    slock<smtx> lock(_mtx);
    return _dist_coeffs;
  }

  int max_features() const;
  double sift_contrast_threshold() const;
  double sift_sigma() const;
  double dist_ratio_in_filter_matches() const;
  int min_matches_edge_in_create_image_graph() const;

  int min_inliers_in_2d2d_matching() const;
  double ransac_confidence_in_compute_H_E() const;
  double max_error_in_compute_F_E() const;
  double max_error_in_compute_H() const;
  double max_error_in_init_triangulte() const;
  double min_angle_in_init_triangulte() const;

  int min_inlers_in_pnp() const;
  int ransac_times_in_pnp() const;
  double ransac_confidence_in_pnp() const;
  double max_error_in_pnp() const;

  double max_error_in_triangulate() const;
  double min_anglue_in_triangulate() const;

  double max_error_in_BA_filter() const;
  double min_angle_in_BA_filter() const;

  int max_image_count_in_localBA() const;
  int max_found_image_count_in_BA_to_run_globalBA() const;
  int max_image_optimized_times() const;

  void setConf(const sp<SysConfig>& conf);

  /// \brief start recording present time point
  /// \param name
  void startTimeRecord(const std::string& name);

  /// \brief end recording present time point, and calculate the time period
  void stopTimeRecord(const std::string& name);

  /// \brief output the time period specified by \p name
  std::string getTimeRecord(const std::string& name) const;

  /// \brief output all time periods ranked by time point in ascending order
  /// \see getTimeRecord
  std::vector<std::string> getFullTimeRecord() const;
};

struct SystemInfo::SysConfig {
  int _max_features;  //< 特征提取最多特征数
  double _sift_contrast_threshold;
  double _sitf_sigma;
  double _dist_ratio_in_filter_matches;         //< 筛选特征点匹配
  int _min_matches_edge_in_create_image_graph;  //< Image Graph
                                                //中，边节点最小的匹配数量

  // 地图初始化时相关参数：
  int _min_inliers_in_2d2d_matching;         //< 2D-2D 对应内点最小数量
  double _ransac_confidence_in_compute_H_E;  //< 求 H、E 时 ransac 置信度
  double _max_error_in_compute_F_E;          //< 求 E 时误差阈值
  double _max_error_in_compute_H;            //< 求 H 时误差阈值
  double _max_error_in_init_triangulte;  //< 三角测量重投影误差阈值
  double _min_angle_in_init_triangulte;  //< 三角测量角度误差阈值

  // Pnp 求解位姿相关参数
  int _min_inlers_in_pnp;             //< Pnp 后 2D-3D 匹配的最小数量
  int _ransac_times_in_pnp;           //< Pnp ransac 迭代次数
  double _ransac_conifidence_in_pnp;  //< Pnp ransac 置信阈值
  double _max_error_in_pnp;           //< Pnp 误差阈值

  // 后续三角测量相关参数
  double _max_error_in_triangulate;   //< 重投影误差阈值
  double _min_anglue_in_triangulate;  //< 最小角度

  // BA 优化后，过滤地图点时的 重投影误差阈值、最小角度
  double _max_error_in_BA_filter;
  double _min_angle_in_BA_filter;

  // Local BA 优化时，用于位姿调整的最大图片数量
  int _max_image_count_in_localBA;
  int _max_found_image_count_in_BA_to_run_globalBA;
  int _max_image_optimized_times;

  SysConfig();
  std::string toString() const;
};

#ifdef MAIN

SystemInfo sys;

#else

extern SystemInfo sys;

#endif  // MAIN

#endif  // !__monocular_system_info_h__
