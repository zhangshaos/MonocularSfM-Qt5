//
/// \file src/model/image.h
///
/// \author zxm
/// \email xingmingzhangssr@gmail
/// \version 0.1
//

#ifndef __monocularsfm_image_h__
#define __monocularsfm_image_h__

#include <atomic>
#include <filesystem>
#include <opencv2/core.hpp>

#include "key_point.h"

struct Image_impl {
  int id = -1;              ///< ID of an image, default -1
  uint64_t R_ba_times = 0;  ///< optimized Rcw times of bundle adjustment
  uint64_t t_ba_times = 0;  ///< optimized tcw times of bundle adjustment
  std::string path;         ///< path of the image file
  cv::Mat gray_mat,
      rgb_mat;                 ///< color or gray mat returned by cv::imread()
  std::vector<KeyPoint> kpts;  ///< all keypoints of this image
  cv::Mat descp;               ///< keypoints' descriptor
  cv::Mat Rcw, tcw;            ///< pose(Tcw = Rcw3x3 | tcw3x1) of this image
};

/// \class Image
/// \brief image of database
/// \note
/// *
class Image {
  sp<Image_impl> self;

 public:
  Image();

  template <typename Archive>
  void serialize(Archive &ar, const unsigned int ver);

  /// \brief get a new Image ID to set image._id
  /// \param none
  /// \return new id
  /// \note
  /// * This funtion can be used in multi-thread
  static void GetNewImageID(int &id);

  void id(int id);
  int id() const;
  const cv::Mat &gray_mat() const;
  void gray_mat(const cv::Mat &m);
  const cv::Mat &rgb_mat() const;
  void rgb_mat(const cv::Mat &m);
  const std::vector<KeyPoint> &kpts() const;
  void kpts(std::vector<KeyPoint> &&kps);
  const cv::Mat &descp() const;
  void descp(const cv::Mat &m);
  const cv::Mat &Rcw() const;
  const cv::Mat &tcw() const;
  void Rcw(const cv::Mat &Rcw);
  void tcw(const cv::Mat &tcw);
  const std::string &path() const;
  void path(const std::string &path);
  uint64_t R_ba_times() const;
  uint64_t t_ba_times() const;
  void inc_R_ba_times();
  void inc_t_ba_times();

  /// \brief get matched map point's id of \p kp_idx-th key point
  /// \param kp_idx
  /// \return matched map point's id
  int64_t getMapptOfKpt(int kp_idx) const;

  /// \brief set matched \p mp_idx-th map point's id of \p kp_idx-th key point
  /// \param kp_idx
  /// \param mp_idx
  void setMapptOfKpt(int kp_idx, int64_t mp_idx);
};

/// \brief Interface of extracting keypoint and computing feature descriptor
/// \note
/// * SIFT
/// * ORB
/// * this interface used in \class DBInit \see db_init.h/cpp for details
struct I_ExtractKeyPoints {
  /// \brief return the \class Image after extracting keypoints and\
  /// computing feature descriptor
  /// \return
  virtual Image run() = 0;
  virtual ~I_ExtractKeyPoints() {}
};

class SIFT_ExtractKeyPoints : public I_ExtractKeyPoints {
  cv::Mat _rgb_mat;
  std::string _path;

 public:
  Image run() override;
  SIFT_ExtractKeyPoints(const cv::Mat &rgb_mat, const std::string &path)
      : _rgb_mat(rgb_mat), _path(path) {}
};

class ORB_ExtractKeyPoints : public I_ExtractKeyPoints {
 public:
  Image run() override;
};

struct I_MatchFeatures {
  /// \brief get match result of two image features
  /// \param[out] matches, matches[i] means the i-th keypoint in first image
  /// matching to the matches[i]-th keypoint in second image. if matches[i] is
  /// -1, meaning no match found for i-th keypoint in first image \return the
  /// count of truly matches
  virtual size_t run(std::vector<cv::DMatch> &matches) = 0;
  virtual ~I_MatchFeatures() {}

  struct I_MatchFilter {
    /// \brief fiter the opencv::DMatch result
    /// \param[in] dms
    /// \return new goood matched result
    /// \retval std::vector<cv::DMatch>
    virtual std::vector<cv::DMatch> filter(
        const std::vector<cv::DMatch> &dms) = 0;
    virtual ~I_MatchFilter() {}
  };
};

class BF_MatchFeatures : public I_MatchFeatures {
  cv::Mat _descp1, _descp2;

 public:
  size_t run(std::vector<cv::DMatch> &matches) override;
  BF_MatchFeatures(const cv::Mat &m1, const cv::Mat &m2)
      : _descp1(m1), _descp2(m2) {}
};

class Flann_Matcher : public I_MatchFeatures {
  cv::Mat _descp1, _descp2;

 public:
  size_t run(std::vector<cv::DMatch> &matches) override;
  Flann_Matcher(const cv::Mat &m1, const cv::Mat &m2)
      : _descp1(m1), _descp2(m2) {}
};

#endif  // !__monocularsfm_image_h__