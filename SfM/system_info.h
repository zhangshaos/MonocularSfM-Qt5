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
  cv::Mat1d _k;                      ///< camera matrix K(fx, fy, cx, cy)
  std::vector<double> _dist_coeffs;  ///< camera disturb
  mutable smtx _mtx;  ///< shared mutex for accessing unusually changed variable

  // statistical info for system running time
  class SysTimeInfo;
  sp<SysTimeInfo> _time;

  // statistical info for other infos
  class SysStatistics;
  sp<SysStatistics> _record;

  // system running config
  struct SysConfig;
  const sp<SysConfig> _conf;

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

#ifdef MAIN

SystemInfo sys;

#else

extern SystemInfo sys;

#endif  // MAIN

#endif  // !__monocular_system_info_h__
