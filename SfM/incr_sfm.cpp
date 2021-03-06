#define _USE_MATH_DEFINES 1

#include <boost/format.hpp>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>

#include "cal_pose.h"
#include "image_graph.h"
#include "map.h"
#include "optimize.h"
#include "re_triangulation.h"
#include "system_info.h"
#include "triangulate.h"

// here
#include "db.h"
#include "incr_sfm.h"

bool IncrSfM::run() {
  using namespace std;
  using Format = boost::format;

  cout << "\n==============================================\n"
          "========START to Increasemental SfM...========"
          "\n==============================================\n";
  constexpr char timer_name[] = "A pass of IncrSfM";
  sys.startTimeRecord(timer_name);
  auto PrintTime = [&timer_name] {
    sys.stopTimeRecord(timer_name);
    cout << sys.getTimeRecord(timer_name);
  };

  auto next_images = db->G().getConnectedOfPart(*used_images);

  int found_images_count = (int)next_images.size(),
      remaining_images_count =
          int(db->images().size() - used_images->size() - 1),
      used_images_count = (int)used_images->size();
  cout << Format(
              "********* Found IncrfSfM images %1%, Remains %2%(exclude this), "
              "Used(Registered) %3% *********\n") %
              found_images_count % remaining_images_count % used_images_count;
  // NOTE:
  // 当 used_images
  // 图片集合刚刚完整收集了一个场景，剩下的图片不足以再补充这个场景时
  // next_images 数量会很少！因此设定当这个数量小于一个阈值时，触发 Global BA！
  // bool is_globalBA =
  //    remaining_images_count >= sys.max_image_count_in_localBA() &&
  //    found_images_count <= sys.max_found_image_count_in_BA_to_run_globalBA();
  bool is_globalBA = false;
  static int global_BA_times = 0;

  int next_image = -1;
  for (int next : next_images) {
    // read a new image to reconstruct map
    // calculate the Pose of image
    cv::Mat1d R(3, 3, 0.), t(3, 1, 0.);
    cv::Mat1i inlier_idxes;
    Image& img = db->images().at(next);
    /* bool t_existed = !(img.Rcw().empty() || img.tcw().empty());
    if (t_existed) {
      R = img.Rcw();
      t = img.tcw();
    } else */
    {
      std::vector<Point3> map_pts;
      std::vector<Point2> kpts;
      trackObservers(next, map_pts, kpts);
      static int old_observer_counts = 0;
      {
        cout << "========= trackObservers() points " << map_pts.size()
             << " =========\n";
        if (2 * (int)map_pts.size() <= old_observer_counts) {
          cout << "============ Triggle Global BA ============\n";
          is_globalBA = true;
          auto ba_param = BAParam::create(db, map, used_images);
          ba_param->addConstRotation(init_image);
          sp<I_Optimize> optimizer(new BAOptimizer(ba_param));
          if (optimizer->optimize()) ba_param->writeBack();
        }
      }
      sp<I_CalculatePose> poser(new PnpPoser(map_pts, kpts, inlier_idxes));
      // TODO: remove the matched relationship between key point outliers and
      // map, which left to BA. points
      if (!poser->calculate(R, t)) {
        cout << Format("====== [%1%] compute pose failed ======\n") % next;
        continue;  // skip
      }
      img.Rcw(R);
      img.tcw(t);
      old_observer_counts = static_cast<int>(map_pts.size());
    }

    // produce all new map points
    CreateNewMapPoints(map, db, *used_images, next);
    if (map->size() >= 50) {
      next_image = next;
    } else {
      cout << Format("====== [%1%] triangulate failed ======\n") % next;
      return false;  // Directly cause IncrSfM failed when global map is clear
    }

    cout
        << boost::format(
               "\n********* IncrSfM: present image %1% *********\nR is\n%2%\nt "
               "is\n%3%\n") %
               img.id() % img.Rcw() % img.tcw();

    // register
    used_images->emplace(next_image);

    // process Local BA
    if (!is_globalBA) {
      auto ba_param = BAParam::create(db, map, used_images, next_image);
      // ba_param->addConstRotation(init_image);
      ba_param->addConstRotation(next_image);
      sp<I_Optimize> optimizer(new BAOptimizer(ba_param));
      if (optimizer->optimize()) {
        ba_param->writeBack();
        std::cout << boost::format(
                         "\n*************** Global Map points: %1% "
                         "***************\n") %
                         map->size();
        ReTriangulator rt(ba_param);
        int n = rt.run();
        cout << Format(
                    "\n********* Re-Triangulate %1% map points *********\n") %
                    n;
        if (is_globalBA) ++global_BA_times;
      }  // BA optimize over
    }

    std::cout
        << boost::format(
               "\n*************** Global Map points: %1% ***************\n") %
               map->size();
    PrintTime();
    return true;  // Okay!!!
  }               // all over !

  // Over!
  if (next_images.empty()) {
    std::cout
        << "===================================\n"
           "=========   Final Infos   =========\n"
           "===================================\n"
        << boost::format(
               "\n************ Global BA times(exclude initializing): %1% "
               "************\n"
               "\n*************** Global Map points: %2% ***************\n") %
               global_BA_times % map->size();
    PrintTime();
  } else {
    // next_images not empty and coming here
    // which means Pnp always failed or global Map points' count it too less
    cout << "\n!!!!!!Pnp always failed or global Map points' count it too "
            "less!!!!!!\n";
  }
  return false;
}

bool IncrSfM::runPnp(int id) {
  using namespace std;
  using Format = boost::format;

  cv::Mat1d R(3, 3, 0.), t(3, 1, 0.);
  cv::Mat1i inlier_idxes;
  Image& img = db->images().at(id);

  do {
    std::vector<Point3> map_pts;
    std::vector<Point2> kpts;
    trackObservers(id, map_pts, kpts);
    // Undistort key points
    do {
      UndistortPoint(kpts, sys.camera_K(), sys.distort_parameter());
    } while (0);
    cout << Format(
                "============ track observer ============\n"
                "* image id: %3%\n"
                "* map points: %1%\n"
                "* observer(2D key points): %2%\n"
                "========================================\n") %
                map_pts.size() % kpts.size() % id;
    // calculate initial R|t to refine Pnp
    bool use_existed_pose = false;
    auto connected = db->G().getAsendingConnected(id);
    for (int id_base : connected) {
      if (used_images->count(id_base) == 0) continue;
      InitialMatchedParam pm(*db, id_base, id);
      auto pts_pair = GetMatchedKeypointsPairFromDB(pm);

      // calculate the relative pose
      {
        cv::Mat1b inliers;
        shared_ptr<I_CalculatePose> cal_poser(
            new EpipolarPoser(pts_pair.first, pts_pair.second, inliers));
        if (!cal_poser->calculate(R, t)) {
          continue;
        }
        // NOTE: if you use oepncv for decompose R-t in epipolar,
        // the t is normalized...
        double scale = 10;  // little trick...
        t *= scale;

        // After calculate relative pose
        // R | t is Rc1c0 | tc1c0, where c0 is id_base, c1 is id
        // Tc1w = Tc1c0 * Tc0w
        // | Rc1c0 tc1c0 | * | Rc0w tc0w | = | Rc1c0 * Rc0w  Rc1c0 * tc0w +
        // tc1c0 | |    0    1   |   |   0    1  |   |   0                     1
        // |
        const Image& img_base = db->images().at(id_base);
        do {
          cout << Format(
                      "******************************\n"
                      "cam0: %1%, cam1: %2%\n"
                      "Rc1c0:\n%3%\n"
                      "tc1c0:\n%4%\n"
                      "******************************\n") %
                      img_base.path() % img.path() % R % t;
        } while (0);
        t = R * img_base.tcw() + t;
        R = R * img_base.Rcw();
        use_existed_pose = true;
        break;
      }
    }  // test all connected images

    {
      // show estimated pose(R|t) and map points
      debugPose(R, t);
      debugPoints(map_pts);
    }

    sp<I_CalculatePose> poser(
        new PnpPoser(map_pts, kpts, inlier_idxes, use_existed_pose));
    if (!poser->calculate(R, t)) {
      cout << Format("====== [%1%] compute pose failed ======\n") % id;
      break;
    }
    img.Rcw(R);
    img.tcw(t);
    return true;
  } while (0);
  return false;
}

void IncrSfM::debugPose(const cv::Mat1d& R, const cv::Mat1d& t) {
  R.convertTo(dbg_Rcw, CV_32F);
  t.convertTo(dbg_tcw, CV_32F);
}

std::pair<cv::Mat1f, cv::Mat1f> IncrSfM::debugPose() const {
  return std::pair<cv::Mat1f, cv::Mat1f>{dbg_Rcw.t(), -dbg_tcw};
}

void IncrSfM::debugPoints(const std::vector<Point3>& pt3s) {
  dbg_pts.clear();
  dbg_pts.reserve(pt3s.size());
  using std::copy;
  using std::inserter;
  copy(pt3s.begin(), pt3s.end(), inserter(dbg_pts, dbg_pts.begin()));
}

std::vector<Point3> IncrSfM::debugPoints() const { return dbg_pts; }

bool IncrSfM::runMapPoints(int id) {
  using namespace std;
  using Format = boost::format;

  // produce all new map points
  CreateNewMapPoints(map, db, *used_images, id);
  if (map->size() < 50) {
    cout << Format(
                "====== [%1%] triangulate failed (global map points are too "
                "few) ======\n") %
                id;
    return false;  // Directly cause IncrSfM failed when global map is clear
  }

  // register
  used_images->emplace(id);
  return true;
}

void IncrSfM::runBA(bool local, bool none_pose, int cur_image) {
  using namespace std;
  using Format = boost::format;
  /**
   * local && none_pose           => create(cur_image, true)
   * local && !none_pose && cur   => create(cur_image, false)
   *                                 addConstant(init_image, cur_image)
   * local && !none_pose && -1    => create(cur_image, false)
   * !local && none_pose          => create(-1, true)
   * !local && !none_pose && cur  => create(-1, false)
   *                                 addConstant(init_image, cur_image)
   * !local && !none_pose && -1   => create(-1, false)
   */
  sp<BAParam> ba_param =
      local ? BAParam::create(db, map, used_images, cur_image, none_pose)
            : BAParam::create(db, map, used_images, -1, none_pose);

  if (!none_pose && cur_image != -1) {
    ba_param->addConstRotation(init_image);
    ba_param->addConstTranslation(init_image);
    ba_param->addConstRotation(cur_image);
    ba_param->addConstTranslation(cur_image);
  }

  // if (ba_param->_poses.size() >= sys.max_image_count_in_localBA()) {
  //  ba_param->setIntrinsicOptimized();
  //}

  if (!local) std::cout << "============ Triggle Global BA ============\n";

  sp<I_Optimize> optimizer(new OptimizerWithIntrinsic(ba_param));
  if (optimizer->optimize()) {
    ba_param->writeBack();
    std::cout << boost::format(
                     "\n*************** Global Map points: %1% "
                     "***************\n") %
                     map->size();
    ReTriangulator rt(ba_param);
    int n = rt.run();
    std::cout << Format(
                     "\n********* Re-Triangulate %1% map points *********\n") %
                     n;
  }
}

void IncrSfM::saveMap(const std::string& path) const {
  map->writeToBinPLY(path);
}

void IncrSfM::savePoses(const std::string& path) const {
  using namespace std;
  using Format = boost::format;
  ofstream file(path);
  assert(file.is_open());

  file << "image-id image-path rotation-vector translation-vector\n";

  for (int id : *used_images) {
    auto& img = db->images().at(id);
    assert(id == img.id());
    cv::Mat1d rot;
    cv::Rodrigues(img.Rcw(), rot);
    file << Format("%1% %2% %3% %4%\n") % id % img.path() % rot.t() %
                img.tcw().t();
  }

  file.close();
}

void IncrSfM::trackObservers(int img, std::vector<Point3>& map_pts,
                             std::vector<Point2>& kpts) const {
  using namespace std;
  unordered_set<int> already_existed;
  map_pts.clear();
  kpts.clear();
  const Image& img1 = db->images().at(img);
  auto connected = db->G().getAllConnected(img);
  for (auto& p : connected) {
    int id = p.first;
    auto& node = p.second;
    if (!used_images->count(id)) continue;
    // handle connected images Ic...
    // If Ic's keypoints matched by current image(id equal \p img)'s keypoints
    // and this keypoints matched map points, then
    // output the keypoints and mappoints to \p map_pts and \p kpts
    const Image& img2 = db->images().at(id);
    for (auto [k1, k2] : node.dmatches) {
      if (already_existed.count(k1)) continue;  // skip
      const KeyPoint& kp2 = img2.kpts().at(k2);
      if (!map->existedMapPoint(kp2.id())) continue;  // skip
      // add a observer of this image \p img!
      const KeyPoint& kp1 = img1.kpts().at(k1);
      kpts.emplace_back(kp1.pt());
      const MapPoint& mp = map->getMapPoint(kp2.id());
      map_pts.emplace_back(mp.pos());
      already_existed.emplace(k1);
      //当 img 的特征点对应好几副图片的对应点时，同一个特征点 -
      //可能不同的3D点对可能被加入！
      //因此设计一个查找表格，如果img的这个特征点已经处理过了，则不再加入
      //// update \var image1 's keypoints' corresponding mappoint's id
      //// and \var mp 's observer 's kypoint's id
      // img1.setMapptOfKpt(k1, mp.id());
      // mp.addObserver(img, k1);
      // NOTE: I don't update these until producing new map points in
      // triangulation, because in this const function, I don't want to update
      // any infos... which making this function owns only one major effort!!!
    }
  }  // over
  assert(already_existed.size() == map_pts.size());
  assert(map_pts.size() == kpts.size());
}

void IncrSfM::getTrackObs(int id, std::vector<int>& kps,
                          std::vector<int64_t>& map_pts) const {
  using namespace std;
  kps.clear();
  map_pts.clear();
  unordered_set<int> already_existed;
  const Image& img1 = db->images().at(id);
  auto connected = db->G().getAllConnected(id);
  for (auto& p : connected) {
    int id2 = p.first;
    auto& node = p.second;
    if (!used_images->count(id2)) continue;
    // handle connected images Ic...
    // If Ic's keypoints matched by current image(id equal \p img)'s keypoints
    // and this keypoints matched map points, then
    // output the keypoints and mappoints to \p map_pts and \p kpts
    const Image& img2 = db->images().at(id2);
    for (auto [k1, k2] : node.dmatches) {
      if (already_existed.count(k1)) continue;  // skip
      int64_t map_pt_id = img2.kpts().at(k2).id();
      if (!map->existedMapPoint(map_pt_id)) continue;  // skip

      kps.emplace_back(k1);
      map_pts.emplace_back(map_pt_id);

      already_existed.emplace(k1);
      //当 img 的特征点对应好几副图片的对应点时，同一个特征点 -
      //可能不同的3D点对可能被加入！
      //因此设计一个查找表格，如果img的这个特征点已经处理过了，则不再加入
      //// update \var image1 's keypoints' corresponding mappoint's id
      //// and \var mp 's observer 's kypoint's id
      // img1.setMapptOfKpt(k1, mp.id());
      // mp.addObserver(img, k1);
      // NOTE: I don't update these until producing new map points in
      // triangulation, because in this const function, I don't want to update
      // any infos... which making this function owns only one major effort!!!
    }
  }  // over
  assert(already_existed.size() == map_pts.size());
  assert(map_pts.size() == kps.size());
}

int IncrSfM::createMapPoints(int id, const std::vector<int>& inliers) {
  using namespace std;
  auto& K = sys.camera_K();
  int count = 0;
  auto& image = db->images().at(id);
  auto& kpts = image.kpts();
  for (int key_pt_i : inliers) {
    auto& kp1 = kpts[key_pt_i];
    if (kp1.empty() || kp1.id() >= 0) continue;  // skip when Re-Triangulating
    auto tracked_image_and_kp_s =
        db->G().getAllTrackedKptsOfPart(id, key_pt_i, *used_images);
    if (tracked_image_and_kp_s.empty()) continue;  // skip
    // add multiple view and key point to \class MultiViewsTriangulater
    vector<cv::Mat> views;
    vector<Point2> pt2s;
    {  // 1. add main tracked views and points...
      cv::Mat1d proj;
      cv::hconcat(image.Rcw(), image.tcw(), proj);
      proj = K * proj;
      views.emplace_back(proj);
      pt2s.emplace_back(kp1.pt());
      // 2. add related tracked view...
      for (auto [other_image, key_pt_i] : tracked_image_and_kp_s) {
        const Image& image2 = db->images().at(other_image);
        auto& kpts2 = image2.kpts();
        const KeyPoint& kp2 = kpts2[key_pt_i];
        int64_t map_pt_id = kp2.id();
        // if the matched KeyPoint alreadly produced a map point:
        // 1. relate this KeyPoint to the map point
        // 2. the map point add a observer about this KeyPoint
        // 3. break triangulating this keypoint
        if (map_pt_id >= 0) {
          image.setMapptOfKpt(key_pt_i, map_pt_id);
          map->getMapPoint(map_pt_id).addObserver(image.id(), key_pt_i);
          goto PRODUCE_NEXT_MAPPT;
        }  // else:
        cv::Mat1d proj2;
        cv::hconcat(image2.Rcw(), image2.tcw(), proj2);
        proj2 = K * proj2;
        views.emplace_back(proj2);
        pt2s.emplace_back(kp2.pt());
      }  // finnish add related views

      // Undistort key points
      do {
        UndistortPoint(pt2s, K, sys.distort_parameter());
      } while (0);

    }  // finish adding all tracked views and points
    {  // start to produce new map point...
      sp<I_Triangulate> tri(new MultiViewsTriangulater(views, pt2s));
      Point3 mp_pt;
      if (!tri->calculate(mp_pt)) continue;

      // insert the map point into global map
      tracked_image_and_kp_s.emplace_back(id, key_pt_i);
      auto mp = MapPoint::create(mp_pt, tracked_image_and_kp_s, db);
      map->addMapPoint(mp);
      ++count;
    }                  // new map point produced over
  PRODUCE_NEXT_MAPPT:  // \see line 61 and this goto-label only used there!!!
    continue;
  }  // check each key points
  return count;
}
