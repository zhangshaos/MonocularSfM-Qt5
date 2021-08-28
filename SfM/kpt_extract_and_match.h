#ifndef __kpt_extract_and_match__
#define __kpt_extract_and_match__
/**
 * extract key points from pictures.
 * and match these key points
 */

class Image;

/**
 * @brief Interface of extracting keypoint and computing feature descriptor
 */
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

/**
 * @brief Do matching for two key points features
 */
struct I_MatchFeatures {
  /// \brief get match result of two image features
  /// \param[out] matches, matches[i] means the i-th keypoint in first image
  /// matching to the matches[i]-th keypoint in second image.
  /// \return the count of truly matches
  virtual size_t run(std::vector<cv::DMatch> &matches) = 0;
  virtual ~I_MatchFeatures() {}
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
  const std::vector<KeyPoint> &_ps1, &_ps2;

 public:
  size_t run(std::vector<cv::DMatch> &matches) override;
  Flann_Matcher(const std::vector<KeyPoint> &ps1, const cv::Mat &m1,
                const std::vector<KeyPoint> &ps2, const cv::Mat &m2);
};

/**
 * @brief 2. filter by nearest/second-nearest ratio
 * @param dms
 * @param ratio
 * @return
 */
std::vector<cv::DMatch> FilterByDistanceRatio(
    const std::vector<std::vector<cv::DMatch>> &dms, float ratio);

/**
 * @brief 3. filter by cross check, <i, j> is a match,
 * only if <i,j> locates in \p dms1 and <j,i> locates in \p dms2
 * @param dms1
 * @param dms2
 * @return
 */
std::vector<cv::DMatch> FilterByCrossCheck(const std::vector<cv::DMatch> &dms1,
                                           const std::vector<cv::DMatch> &dms2);

/**
 * @brief 5. filter by epipolar constraint F or H
 * @param kps1
 * @param kps2
 * @param dms
 * @param use_homogrphy
 */
void FilterByEpipolarConstraint(const std::vector<KeyPoint> &kps1,
                                const std::vector<KeyPoint> &kps2,
                                std::vector<cv::DMatch> &dms,
                                bool use_homogrphy);

#endif
