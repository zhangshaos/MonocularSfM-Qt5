#include "triangulate.h"

#include <boost/format.hpp>
#include <opencv2/opencv.hpp>

#include "algorithm.h"
#include "map_point_filter.h"
#include "system_info.h"

bool TwoViewTriangulater::calculate(Point3& pt) {
  using namespace std;
  cv::Mat proj1, proj2;
  cv::hconcat(_R1, _t1, proj1);
  proj1 = _K * proj1;
  cv::hconcat(_R2, _t2, proj2);
  proj2 = _K * proj2;
  std::vector<Point2> v1(1, _pt1), v2(1, _pt2);
  cv::Mat pt4s(4, 1, CV_64F);
  cv::triangulatePoints(proj1, proj2, v1, v2, pt4s);

  double ratio = 1.0 / pt4s.at<double>(3, 0);
  pt.x = pt4s.at<double>(0, 0) * ratio;
  pt.y = pt4s.at<double>(1, 0) * ratio;
  pt.z = pt4s.at<double>(2, 0) * ratio;

  // start to filter...
  // 1. filter according to depth
  // 2. check reprojection error
  // 3. check the angle of triangulation
  vector<cv::Mat> projs{proj1, proj2};
  sp<I_MapPointFilter> depth_test(new ReprojDistanceFilter(10 * 100, projs, 2));
  if (depth_test->isBad(pt)) {
    return false;
  }
  vector<Point2> pt2s{_pt1, _pt2};
  sp<I_MapPointFilter> reproj_test(new ReprojErrorFilter(
      sys.max_error_in_init_triangulte(), pt2s, projs, 2));
  if (reproj_test->isBad(pt)) {
    return false;
  }
  sp<I_MapPointFilter> angle_test(
      new AngleFilter(sys.min_angle_in_init_triangulte(), _R1, _R2, _t1, _t2));
  if (angle_test->isBad(pt)) {
    return false;
  }
  return true;
}

bool MultiViewsTriangulater::calculate(Point3& pt) {
  // algorithm refer to email: xingmingzhangssr@gmail
  cv::Mat1d J(4, 4, 0.);
  assert(_projs.size() == _pts.size());
  for (int i = 0; i < _projs.size(); ++i) {
    double u = _pts[i].x, v = _pts[i].y;
    auto& proj = _projs[i];
    cv::Mat r1 = proj.row(0), r2 = proj.row(1), r3 = proj.row(2);
    cv::Mat j1 = u * r3 - r1, j2 = v * r3 - r2;
    J += j1.t() * j1;
    J += j2.t() * j2;
  }
  // SVD
  cv::Mat1d U, S, V;
  cv::SVD::compute(J, S, U, V, cv::SVD::Flags::MODIFY_A);
  double minmum = S.at<double>(3, 0), second_minmum = S.at<double>(2, 0);
  if (second_minmum < minmum * 1'000.) return false;  // industry trick
  cv::Mat1d h_pos = V.row(3);
  // NOTE: if J is full-ranked(JX = 0 no answer), which lead to h_pos = Zeor
  // we must to filter this case
  h_pos /= h_pos.at<double>(0, 3);
  pt.x = h_pos.at<double>(0, 0);
  pt.y = h_pos.at<double>(0, 1);
  pt.z = h_pos.at<double>(0, 2);

  if (abs(pt.x) <= 1e-8 && abs(pt.y) <= 1e-8 && abs(pt.z) <= 1e-8) {
    std::cout << "\nSVD full rank\n" << std::endl;
    return false;
  }

  const int min_okay_reproj_count =
      std::max(static_cast<int>(_projs.size()) - 1, 2);

  sp<I_MapPointFilter> depth_test(
      new ReprojDistanceFilter(10 * 50, _projs, min_okay_reproj_count));
  if (depth_test->isBad(pt)) {
    return false;
  }

  sp<I_MapPointFilter> reproj_test(new ReprojErrorFilter(
      sys.max_error_in_triangulate(), _pts, _projs, min_okay_reproj_count));
  if (reproj_test->isBad(pt)) {
    return false;
  }

  // NOTE: \see incr_sfm.cpp => IncfSfM::run() => CreateMapPoints() => \var
  // views and \var kp2s you could found that \a _pts, \a _projs are in
  // ascending order with key-point-matches, so you can measure their angle
  // between each other
  // sp<I_MapPointFilter> angle_test(new AngleFilter(
  // sys.min_anglue_in_triangulate(), ))

  return true;
}
