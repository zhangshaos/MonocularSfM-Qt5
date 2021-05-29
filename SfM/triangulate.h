//
/// \file src/control/triangulate.h
///
/// \author zxm
/// \email xingmingzhangssr@gmail
/// \version 0.1
//

#ifndef __monocular_triangulate_h__
#define __monocular_triangulate_h__

#include <vector>

#include "common_type.h"

/// \brief produce new map point from transforming pose
struct I_Triangulate {
  /// \brief triangulate for producing new map point
  /// \param[out] pt
  /// \return TRUE means ok, FALSE means triangulating failed
  virtual bool calculate(Point3 &pt) = 0;
  virtual ~I_Triangulate() {}
};

class TwoViewTriangulater : public I_Triangulate {
  const cv::Mat &_R1, &_t1, &_R2, &_t2, &_K;
  const Point2 &_pt1, &_pt2;

 public:
  TwoViewTriangulater(const cv::Mat &R1, const cv::Mat &t1, const cv::Mat &R2,
                      const cv::Mat &t2, const cv::Mat &K, const Point2 &pt1,
                      const Point2 &pt2)
      : _R1(R1), _R2(R2), _t1(t1), _t2(t2), _K(K), _pt1(pt1), _pt2(pt2) {}

  /// \brief triangulate for producing new map point
  /// \param[out] pt
  /// \return TRUE means ok, FALSE means triangulating failed
  bool calculate(Point3 &pt) override;
};

class MultiViewsTriangulater : public I_Triangulate {
  const std::vector<cv::Mat> &_projs;
  const std::vector<Point2> &_pts;

 public:
  /// \brief add project matrixand a key point position of X-th view
  /// \param projs (Rcw | tcw)3x4 of X-th view
  /// \param pts a key point position of X-th view
  /// \note
  /// * all input parameter are reference, you must be 
  MultiViewsTriangulater(const std::vector<cv::Mat> &projs,
                         const std::vector<Point2> &pts)
      : _projs(projs), _pts(pts) {}

  /// \brief triangulate for producing new map point
  /// \param[out] pt
  /// \return TRUE means ok, FALSE means triangulating failed
  bool calculate(Point3 &pt) override;
};

#endif  // !__monocular_triangulate_h__
