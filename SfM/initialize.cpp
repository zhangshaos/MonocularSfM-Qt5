#define _USE_MATH_DEFINES
#include "initialize.h"

#include <algorithm>
#include <atomic>
#include <execution>
#include <opencv2/opencv.hpp>
#include <string>

#include "cal_pose.h"
#include "global_config.h"
#include "key_point.h"
#include "map.h"
#include "optimize.h"
#include "system_info.h"
#include "triangulate.h"

// DEBUG
// convert KeyPoints to cv::KeyPoints
inline std::vector<cv::KeyPoint> convertToCVKps(
    const std::vector<KeyPoint> &kps) {
  std::vector<cv::KeyPoint> res;
  for (auto &kp : kps) {
    res.emplace_back(KeyPoint::ToCVKp(kp));
  }
  return res;
}

/// \brief src/control/initialize.h
/// \ref GetMatchedKeypointsPairFromDB(...) parameter
struct InitialMatchedParam {
  const DB &db;
  int pr1, pr2;
  std::vector<int> ptIdxes1, ptIdxes2;
  /// \brief constructor
  /// \param _db
  /// \param img1
  /// \param img2
  InitialMatchedParam(const DB &_db, int img1, int img2)
      : db(_db), pr1(img1), pr2(img2) {}
};

inline std::pair<std::vector<Point2>, std::vector<Point2>>
GetMatchedKeypointsPairFromDB(InitialMatchedParam &pm) {
  auto &dms = pm.db.G().getAllConnected(pm.pr1).at(pm.pr2).dmatches;
  auto &kpts1 = pm.db.images().at(pm.pr1).kpts(),
       &kpts2 = pm.db.images().at(pm.pr2).kpts();

  // fill into retrun val
  std::vector<Point2> v1(dms.size()), v2(dms.size());
  pm.ptIdxes1.resize(dms.size()), pm.ptIdxes2.resize(dms.size());
  int i = 0;
  smtx i_mtx;
  std::for_each(std::execution::par, dms.begin(), dms.end(),
                [&](const std::pair<int, int> &dm) {
                  ulock<smtx> lock(i_mtx);
                  int j = i++;
                  lock.unlock();
                  pm.ptIdxes1[j] = dm.first;
                  pm.ptIdxes2[j] = dm.second;
                  v1[j] = kpts1[dm.first].pt();
                  v2[j] = kpts2[dm.second].pt();
                });
  // test
  assert(i == v2.size());
  assert(i == v1.size());
  assert(i == pm.ptIdxes2.size());
  assert(i == pm.ptIdxes1.size());
  return {v1, v2};
}

bool Initialize::run(sp<Map> &map) {
  using namespace std;
  using Format = boost::format;

  cout << "\n==============================================\n"
          "========       Initialize SfM...      ========"
          "\n==============================================\n";

  // set timer
  constexpr char timer_name[] = "initialize the map";
  sys.startTimeRecord(timer_name);
  auto PrintTimer = [&timer_name] {
    sys.stopTimeRecord(timer_name);
    cout << sys.getTimeRecord(timer_name);
  };

  map.reset(new Map);
  auto matched_pairs = _db->G().getAllMatchedPair();

  for (auto [pr1, pr2] : matched_pairs) {
    // get best matching image pair in global database
    InitialMatchedParam pm(*_db, pr1, pr2);
    auto pts_pair = GetMatchedKeypointsPairFromDB(pm);

    // calculate the pose
    cv::Mat1d R1 = cv::Mat1d::eye(3, 3), t1 = cv::Mat1d::zeros(3, 1),
              R2(3, 3, 0.), t2(3, 1, 0.);
    cv::Mat1b inliers;
    Image &img1 = _db->images().at(pm.pr1), &img2 = _db->images().at(pm.pr2);
    /* bool t_existed = !(img1.Rcw().empty() || img1.tcw().empty() ||
                       img2.Rcw().empty() || img2.tcw().empty());
    if (t_existed) {
      // the Poses \var image1 and \var image2 are both existed
      R1 = img1.Rcw();
      t1 = img1.tcw();
      R2 = img2.Rcw();
      t2 = img2.tcw();
    } else */
    {
      // you need to calculate the pose...
      shared_ptr<I_CalculatePose> cal_poser(
          new EpipolarPoser(pts_pair.first, pts_pair.second, inliers));
      if (!cal_poser->calculate(R2, t2)) {
        cout << Format("====== [%1%, %2%] compute pose failed ======\n") %
                    pm.pr1 % pm.pr2;
        continue;
      }
      // NOTE: if you use oepncv for decompose R-t in epipolar, the t is
      // normalized...
      double scale = 1'0000;  // little trick...
      t1 *= scale;
      t2 *= scale;
      img1.Rcw(R1);
      img1.tcw(t1);
      img2.Rcw(R2);
      img2.tcw(t2);
    }

    assert(inliers.rows == pts_pair.first.size());
    assert(inliers.rows == pm.ptIdxes1.size());

    // produce new map points
    auto CreateMapPoints = [&]() {
      for (int i = 0; i < inliers.rows; ++i) {
        if (inliers.at<uchar>(i, 0) == 0) continue;  // skip outliers
        // triangulate...
        const Point2 &pt2 = pts_pair.second[i], &pt1 = pts_pair.first[i];
        sp<I_Triangulate> tri(
            new TwoViewTriangulater(R1, t1, R2, t2, sys.camera_K(), pt1, pt2));
        Point3 mappt;
        if (!tri->calculate(mappt)) continue;
        // insert the map point into global map
        vector<pair<int, int>> obs{{pm.pr1, pm.ptIdxes1[i]},
                                   {pm.pr2, pm.ptIdxes2[i]}};
        auto mp = MapPoint::create(mappt, obs, _db);
        map->addMapPoint(mp);
      }
    };
    CreateMapPoints();
    if (map->size() < 50) {
      map->clear(_db);
      cout << Format("====== [%1%, %2%] triangulate failed ======\n") % pm.pr1 %
                  pm.pr2;
      continue;
    }

    // register
    _used_images->emplace(pm.pr1);
    _used_images->emplace(pm.pr2);
    _init_image = pm.pr1;

    std::cout << boost::format(
                     "**************************************\n"
                     "*** Initialized Pair is [%1%, %2%] ***\n"
                     "**************************************\n") %
                     pm.pr1 % pm.pr2;

    // process a pass of global ba
    auto ba_param = BAParam::create(_db, map, _used_images);
    ba_param->addConstPose(_init_image);
    sp<I_Optimize> optimizer(new BAOptimizer(ba_param));
    if (optimizer->optimize()) ba_param->writeBack();

    std::cout
        << boost::format(
               "\n*************** Global Map points: %1% ***************\n") %
               map->size();
    PrintTimer();
    return true;
  }
  PrintTimer();
  return false;
}
