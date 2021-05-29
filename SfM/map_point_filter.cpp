#include "map_point_filter.h"

inline bool AllowedReprojectError(double allowed_error, const Point3& pt3,
                                  const Point2& pt2, const cv::Mat& proj) {
  using namespace std;
  cv::Mat1d hpt3(4, 1);
  hpt3 << pt3.x, pt3.y, pt3.z, 1.;
  cv::Mat huv = proj * hpt3;
  Point2 reproj_pt2(huv.at<double>(0, 0) / huv.at<double>(2, 0),
                    huv.at<double>(1, 0) / huv.at<double>(2, 0));
  //{
  //  Print(boost::format("original pt2 %1%, newer reproj-pt2 %2%\n") % pt2 %
  //        reproj_pt2);
  //}
  double dif_x = pt2.x - reproj_pt2.x, dif_y = pt2.y - reproj_pt2.y;
  return dif_x * dif_x + dif_y * dif_y <= allowed_error * allowed_error;
}

bool ReprojErrorFilter::isBad(const Point3& pt3)
{
  int okay = 0;
  // TODO: to parallel
  for (int i = 0; i < _pt2s.size(); ++i)
    if (AllowedReprojectError(_allowed_error, pt3, _pt2s[i], _proj[i])) ++okay;
  return okay < _min_okay_proj_count;
}

inline double TriangulationAngle(const cv::Mat& R1, const cv::Mat& R2,
                                 const cv::Mat& t1, const cv::Mat& t2,
                                 const Point3& pt3) {
  // { R1, t1 } => Tcw1
  // R1 * center1 + t1 => (0,0,0),
  // center1 = R1.t() * (-t1)
  cv::Mat center1 = -R1.t() * t1, center2 = -R2.t() * t2;
  cv::Vec3d c1, c2;
  center1.col(0).copyTo(c1);
  center2.col(0).copyTo(c2);

  cv::Vec3d v1 = cv::Vec3d(pt3) - c1, v2 = cv::Vec3d(pt3) - c2;
  double n1 = sqrt(v1.dot(v1)), n2 = sqrt(v2.dot(v2));
  return acos(v1.dot(v2) / (n1 * n2));
}

bool AngleFilter::isBad(const Point3& pt3)
{
  double ang = TriangulationAngle(_R1, _R2, _t1, _t2, pt3);
  return ang < _min_angle_in_radian;
}
