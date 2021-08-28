#define _USE_MATH_DEFINES 1

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vector>

#include "image.h"
#include "image_graph.h"
#include "system_info.h"
#include "triangulate.h"

// here
#include "db.h"
#include "re_triangulation.h"

ReTriangulator::ReTriangulator(const sp<BAParam>& param) : _data(param) {}

ReTriangulator::~ReTriangulator() {}

// TODO: 根据 Pnp 的 inlier_indices 来设置 kpts
int CreateNewMapPoints(sp<Map>& map, sp<DB>& db, const UsedImages& part_images,
                       int cur_image) {
  using namespace std;
  auto& K = sys.camera_K();
  int count = 0;
  auto& image = db->images().at(cur_image);
  auto& kpts = image.kpts();
  for (int i = 0; i < kpts.size(); ++i) {
    auto& kp1 = kpts[i];
    if (kp1.empty() || kp1.id() >= 0) continue;  // skip when Re-Triangulating
    auto tracked_image_and_kp_s =
        db->G().getAllTrackedKptsOfPart(cur_image, i, part_images);
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
          image.setMapptOfKpt(i, map_pt_id);
          map->getMapPoint(map_pt_id).addObserver(image.id(), i);
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
      tracked_image_and_kp_s.emplace_back(cur_image, i);
      auto mp = MapPoint::create(mp_pt, tracked_image_and_kp_s, db);
      map->addMapPoint(mp);
      ++count;
    }                  // new map point produced over
  PRODUCE_NEXT_MAPPT:  // \see line 61 and this goto-label only used there!!!
    continue;
  }  // check each key points
  return count;
}

int ReTriangulator::run() const {
  int count = 0;
  // how to create new map points for some connected images...
  // logic is similar to CreateMapPoints() in IncrSfM.cpp,
  // but the next_image is each one in \a _data->_poses...
  UsedImages part_images;
  for (auto& p : _data->_poses) part_images.emplace(p.first);
  for (int image : part_images) {
    int n = CreateNewMapPoints(_data->_map, _data->_db, part_images, image);
    count += n;
  }
  return count;
}
