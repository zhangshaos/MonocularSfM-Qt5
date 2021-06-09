
#ifndef __monocular_map_point_filter_h__
#define __monocular_map_point_filter_h__

#include "common_type.h"

struct I_MapPointFilter {
  virtual bool isBad(const Point3& pt3) = 0;
  virtual ~I_MapPointFilter() {}
};

class ReprojErrorFilter : public virtual I_MapPointFilter {
  double _allowed_error;
  int _min_okay_proj_count;
  const std::vector<Point2>& _pt2s;
  const std::vector<cv::Mat>& _proj;

 public:
  ReprojErrorFilter(double allowed_error, const std::vector<Point2>& pt2s,
                    const std::vector<cv::Mat>& proj, int min_okay_proj_count)
      : _allowed_error(allowed_error),
        _min_okay_proj_count(min_okay_proj_count),
        _pt2s(pt2s),
        _proj(proj) {}
  bool isBad(const Point3& pt3) override;
};

class AngleFilter : public virtual I_MapPointFilter {
  double _min_angle_in_radian;
  const cv::Mat& _R1;
  const cv::Mat& _R2;
  const cv::Mat& _t1;
  const cv::Mat& _t2;

 public:
  AngleFilter(double min_angle_in_radian, const cv::Mat& R1, const cv::Mat& R2,
              const cv::Mat& t1, const cv::Mat& t2)
      : _min_angle_in_radian(min_angle_in_radian),
        _R1(R1),
        _R2(R2),
        _t1(t1),
        _t2(t2) {}
  bool isBad(const Point3& pt3) override;
};

class ReprojDistanceFilter : public virtual I_MapPointFilter {
  double _max_dist;
  int _min_okay_proj_count;
  const std::vector<cv::Mat>& _proj;

 public:
  ReprojDistanceFilter(double max_dist, const std::vector<cv::Mat>& proj,
                    int min_okay_proj_count)
      : _max_dist(max_dist),
        _min_okay_proj_count(min_okay_proj_count),
        _proj(proj) {}
  bool isBad(const Point3& pt3) override;
};

#endif
