#include <opencv2/calib3d.hpp>
#include <random>
#include <unordered_map>
#include <vector>

// here
#include "image.h"
#include "kpt_extract_and_match.h"
#include "system_info.h"

namespace {

// 1. set the max features of SIFT keypoins
std::vector<cv::KeyPoint> LimitSIFTkeypointsByLayer(
    int nfeatures, const std::vector<cv::KeyPoint>& kps) {
  using namespace std;
  if (nfeatures >= kps.size()) return kps;

  // classify the keypoints by their octave
  unordered_map<int8_t, vector<int>> o_to_idxes;
  for (int i = 0; i < kps.size(); ++i) {
    int8_t o = kps[i].octave & 0xff;
    o_to_idxes[o].emplace_back(i);
  }
  int sorts = (int)o_to_idxes.size();

  // fill the filtered keypoins
  double r = double(nfeatures) / double(kps.size());
  int remained = nfeatures;
  vector<cv::KeyPoint> filtered_kps;
  for (int o = -1; o < sorts - 1; ++o) {
    int N = cvRound(o_to_idxes.at(o).size() * r), n = N;

    if (N > remained) n = remained;

    for (int i = 0; i < n; ++i) {
      auto& kp = kps.at(o_to_idxes.at(o).at(i));
      filtered_kps.emplace_back(kp);
    }
    remained -= n;
  }
  return filtered_kps;
}

// 2. limit the count of SIFT keypoints by TopScale...
std::vector<cv::KeyPoint> LimitSIFTkeypointsByTopScale(
    int nfeatures, std::vector<cv::KeyPoint>& kps) {
  if (nfeatures >= kps.size()) return kps;

  nth_element(kps.begin(), kps.begin() + nfeatures, kps.end(),
              [](const cv::KeyPoint& k1, const cv::KeyPoint& k2) {
                return k1.size > k2.size;
              });

  return std::vector<cv::KeyPoint>(kps.begin(), kps.begin() + nfeatures);
}

// 3. limit the count of SIFT keypoints by Random Sample...
std::vector<cv::KeyPoint> LimitSIFTkeypointsByRandomSample(
    int nfeatures, const std::vector<cv::KeyPoint>& kps) {
  if (nfeatures >= kps.size()) return kps;

  std::vector<cv::KeyPoint> filtered;
  std::sample(kps.begin(), kps.end(), std::back_inserter(filtered), nfeatures,
              std::mt19937(std::random_device()()));

  assert(filtered.size() == nfeatures);
  return filtered;
}

}  // namespace

Image SIFT_ExtractKeyPoints::run() {
  using namespace std;
  // never use \p nfeatures to limit SIFT features number!!!
  auto sift = cv::SIFT::create(0, 3, sys.sift_contrast_threshold(), 10.0,
                               sys.sift_sigma());

  vector<cv::KeyPoint> kps;
  sift->detect(_rgb_mat, kps);
  kps = LimitSIFTkeypointsByTopScale(sys.max_features(), kps);
  cv::Mat descp;
  sift->compute(_rgb_mat, kps, descp);

  Image img;
  int new_id = 0;
  Image::GetNewImageID(new_id);
  img.id(new_id);
  img.kpts(KeyPoint::TransformFromCVKeypoints(kps));
  img.descp(descp);
  img.rgb_mat(_rgb_mat);
  img.path(_path);

  return img;
}

Image ORB_ExtractKeyPoints::run() { return Image(); }

size_t BF_MatchFeatures::run(std::vector<cv::DMatch>& matches) {
  cv::BFMatcher bf(cv::NORM_L2, true);

  return matches.size();
}

Flann_Matcher::Flann_Matcher(const std::vector<KeyPoint>& ps1,
                             const cv::Mat& m1,
                             const std::vector<KeyPoint>& ps2,
                             const cv::Mat& m2)
    : _descp1(m1), _descp2(m2), _ps1(ps1), _ps2(ps2) {}

size_t Flann_Matcher::run(std::vector<cv::DMatch>& matches) {
  using namespace std;
  cv::FlannBasedMatcher flann;
  vector<cv::DMatch> dms1, dms2;
  flann.match(_descp1, _descp2, dms1);
  flann.match(_descp2, _descp1, dms2);
  FilterByCrossCheck(dms1, dms2).swap(matches);
  FilterByEpipolarConstraint(_ps1, _ps2, matches, true);
  return matches.size();
}

//**************************************************************//
//                    match filter operations                   //
//**************************************************************//

// 2. filter by nearest/second-nearest ratio
std::vector<cv::DMatch> FilterByDistanceRatio(
    const std::vector<std::vector<cv::DMatch>>& dms, float ratio) {
  std::vector<cv::DMatch> good;
  for (auto& dm : dms) {
    if (dm.empty()) continue;
    if (dm.size() == 1) {
      good.emplace_back(dm[0]);
      continue;
    }
    if (dm[0].distance <= ratio * dm[1].distance) good.emplace_back(dm[0]);
  }
  return good;
}

// 3. filter by cross check, <i, j> is a match,
// only if <i,j> locates in \p dms1 and <j,i> locates in \p dms2
std::vector<cv::DMatch> FilterByCrossCheck(
    const std::vector<cv::DMatch>& dms1, const std::vector<cv::DMatch>& dms2) {
  using namespace std;
  std::vector<cv::DMatch> good;
  struct match {
    int idx;
    float distance;
  };
  unordered_map<int, match> matched_idxes;
  // matched_idxes[i] means KeyPoint-i matched KP-matched_idxes[i].idx
  // and their distance is matched_idxes[i].distance
  matched_idxes.reserve(dms2.size());
  for (auto& dm : dms2) {
    int j = dm.queryIdx, i = dm.trainIdx;
    if (!matched_idxes.count(i) || matched_idxes[i].distance > dm.distance)
      matched_idxes[i] = match{j, dm.distance};
  }
  for (auto& dm : dms1) {
    int i = dm.queryIdx, j = dm.trainIdx;
    if (matched_idxes.count(i) && matched_idxes[i].idx == j)
      good.emplace_back(dm);
  }
  return good;
}

// 5. filter by epipolar constraint F or H
void FilterByEpipolarConstraint(const std::vector<KeyPoint>& kps1,
                                const std::vector<KeyPoint>& kps2,
                                std::vector<cv::DMatch>& dms,
                                bool use_homogrphy) {
  using namespace std;
  using namespace cv;
  vector<Point2> pts1, pts2;
  for (auto& dm : dms) {
    int i = dm.queryIdx, j = dm.trainIdx;
    pts1.emplace_back(kps1[i].pt());
    pts2.emplace_back(kps2[j].pt());
  }
  const double confidence = sys.ransac_confidence_in_compute_H_E(),
               error_F = sys.max_error_in_compute_F_E(),
               error_H = sys.max_error_in_compute_H();
  Mat inliers_F;
  findFundamentalMat(pts1, pts2, FM_RANSAC, error_F, confidence, inliers_F);
  Mat inliers_H;
  if (use_homogrphy) {
    findHomography(pts1, pts2, RANSAC, error_H, inliers_H, 1000, confidence);
  }

  vector<DMatch> ans;
  for (int i = 0, end = dms.size(); i < end; ++i) {
    if (inliers_F.at<uchar>(i) == 0 &&
        ((use_homogrphy && inliers_H.at<uchar>(i) == 0) || !use_homogrphy)) {
      continue;
    }
    ans.emplace_back(dms[i]);
  }
  swap(ans, dms);
}
