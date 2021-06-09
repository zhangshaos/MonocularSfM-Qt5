#include "image.h"

#include <opencv2/opencv.hpp>
#include <random>
#include <unordered_map>

#include "system_info.h"

// 1. set the max features of SIFT keypoins
inline std::vector<cv::KeyPoint> LimitSIFTkeypointsByLayer(
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
inline std::vector<cv::KeyPoint> LimitSIFTkeypointsByTopScale(
    int nfeatures, std::vector<cv::KeyPoint>& kps) {
  if (nfeatures >= kps.size()) return kps;

  nth_element(kps.begin(), kps.begin() + nfeatures, kps.end(),
              [](const cv::KeyPoint& k1, const cv::KeyPoint& k2) {
                return k1.size > k2.size;
              });

  return std::vector<cv::KeyPoint>(kps.begin(), kps.begin() + nfeatures);
}

// 3. limit the count of SIFT keypoints by Random Sample...
inline std::vector<cv::KeyPoint> LimitSIFTkeypointsByRandomSample(
    int nfeatures, const std::vector<cv::KeyPoint>& kps) {
  if (nfeatures >= kps.size()) return kps;

  std::vector<cv::KeyPoint> filtered;
  std::sample(kps.begin(), kps.end(), std::back_inserter(filtered), nfeatures,
              std::mt19937(std::random_device()()));

  assert(filtered.size() == nfeatures);
  return filtered;
}

Image SIFT_ExtractKeyPoints::run() {
  using namespace std;
  // never use \p nfeatures to limit SIFT features number!!!
  auto sift = cv::SIFT::create(0, 3, 0.03, 10, 1.25);

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

// 1. fiter by global distance threshold( bad results )
inline std::vector<cv::DMatch> FilterMatchedResult(
    const std::vector<cv::DMatch>& dms) {
  using namespace std;
  float max_dist = 0.f, min_dist = numeric_limits<float>::max();
  for (const auto& dm : dms) {
    max_dist = max(max_dist, dm.distance);
    min_dist = min(min_dist, dm.distance);
  }
  vector<cv::DMatch> res;
  for (const auto& dm : dms) {
    if (dm.distance <= (min_dist + max_dist) * 0.67f) res.emplace_back(dm);
  }
  return res;
}

// 2. filter by nearest/second-nearest ratio
inline std::vector<cv::DMatch> FilterMatchedResult(
    const std::vector<std::vector<cv::DMatch>>& dms) {
  std::vector<cv::DMatch> good;
  for (auto& dm : dms) {
    if (dm[0].distance <= sys.dist_ratio_in_filter_matches() * dm[1].distance)
      good.emplace_back(dm[0]);
  }
  return good;
}

// 3. filter by cross check, <i, j> is a match,
// only if <i,j> locates in \p dms1 and <j,i> locates in \p dms2
inline std::vector<cv::DMatch> CrossCheck(const std::vector<cv::DMatch>& dms1,
                                          const std::vector<cv::DMatch>& dms2) {
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

size_t BF_MatchFeatures::run(std::vector<cv::DMatch>& matches) {
  cv::BFMatcher bf(cv::NORM_L2, true);
  std::vector<cv::DMatch> res;
  bf.match(_descp1, _descp2, res);
  auto good_res = FilterMatchedResult(res);
  std::swap(good_res, matches);
  return matches.size();
}

size_t Flann_Matcher::run(std::vector<cv::DMatch>& matches) {
  using namespace std;
  cv::FlannBasedMatcher flann;
  vector<vector<cv::DMatch>> kdms1, kdms2;
  flann.knnMatch(_descp1, _descp2, kdms1, 2);
  flann.knnMatch(_descp2, _descp1, kdms2, 2);
  auto dms1 = FilterMatchedResult(kdms1), dms2 = FilterMatchedResult(kdms2);
  matches = CrossCheck(dms1, dms2);
  return matches.size();
}

Image::Image() : self(new Image_impl) {}

void Image::GetNewImageID(int& id) {
  static mtx m;
  static int _id_ = 0;
  ulock<mtx> lock(m);
  id = _id_++;
}

void Image::id(int id) { self->id = id; }

int Image::id() const { return self->id; }

const cv::Mat& Image::gray_mat() const { return self->gray_mat; }

void Image::gray_mat(const cv::Mat& m) { self->gray_mat = m; }

const cv::Mat& Image::rgb_mat() const { return self->rgb_mat; }

void Image::rgb_mat(const cv::Mat& m) { self->rgb_mat = m; }

const std::vector<KeyPoint>& Image::kpts() const { return self->kpts; }

void Image::kpts(std::vector<KeyPoint>&& kps) { self->kpts = std::move(kps); }

const cv::Mat& Image::descp() const { return self->descp; }

void Image::descp(const cv::Mat& m) { self->descp = m; }

const cv::Mat& Image::Rcw() const { return self->Rcw; }

const cv::Mat& Image::tcw() const { return self->tcw; }

void Image::Rcw(const cv::Mat& Rcw) { self->Rcw = Rcw; }

void Image::tcw(const cv::Mat& tcw) { self->tcw = tcw; }

const std::string& Image::path() const { return self->path; }

void Image::path(const std::string& path) { self->path = path; }

uint64_t Image::R_ba_times() const { return self->R_ba_times; }

uint64_t Image::t_ba_times() const { return self->t_ba_times; }

void Image::inc_R_ba_times() { ++self->R_ba_times; }

void Image::inc_t_ba_times() { ++self->t_ba_times; }

int64_t Image::getMapptOfKpt(int kp_idx) const {
  return kpts().at(kp_idx).id();
}

void Image::setMapptOfKpt(int kp_idx, int64_t mp_idx) {
  self->kpts.at(kp_idx).id(mp_idx);
}
