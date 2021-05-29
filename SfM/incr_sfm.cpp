#define _USE_MATH_DEFINES 1
#include "incr_sfm.h"

#include <fstream>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>

#include "cal_pose.h"
#include "map.h"
#include "optimize.h"
#include "re_triangulation.h"
#include "system_info.h"
#include "triangulate.h"

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
          ba_param->addConstPose(init_image);
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
      old_observer_counts = (int)map_pts.size();
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
      // ba_param->addConstPose(init_image);
      ba_param->addConstPose(next_image);
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
