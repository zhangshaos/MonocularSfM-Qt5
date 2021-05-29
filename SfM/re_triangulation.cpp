#define _USE_MATH_DEFINES 1
#include "re_triangulation.h"

#include <opencv2/opencv.hpp>
#include <vector>

#include "system_info.h"
#include "triangulate.h"

ReTriangulator::ReTriangulator(const sp<BAParam>& param) : _data(param) {}

ReTriangulator::~ReTriangulator() {}

int CreateNewMapPoints(sp<Map>& map, sp<DB>& db, const UsedImages& part_images,
                       int cur_image) {
  using namespace std;
  int count = 0;
  auto& image = db->images().at(cur_image);
  auto& kpts = image.kpts();
  for (int i = 0; i < kpts.size(); ++i) {
    auto& kp1 = kpts[i];
    if (kp1.id() >= 0) continue;  // skip when Re-Triangulating
    auto tracked_image_and_kp =
        db->G().getAllTrackedKptsOfPart(cur_image, i, part_images);
    if (tracked_image_and_kp.empty()) continue;  // skip
    // add multiple view and key point to \class MultiViewsTriangulater
    vector<cv::Mat> views;
    vector<Point2> pt2s;
    {  // 1. add main tracked views and points...
      cv::Mat1d proj;
      cv::hconcat(image.Rcw(), image.tcw(), proj);
      proj = sys.camera_K() * proj;
      views.emplace_back(proj);
      pt2s.emplace_back(kp1.pt());
      // 2. add related tracked view...
      for (auto [image2_id, kp2_idx] : tracked_image_and_kp) {
        const Image& image2 = db->images().at(image2_id);
        auto& kpts2 = image2.kpts();
        const KeyPoint& kp2 = kpts2[kp2_idx];
        int64_t mp_id = kp2.id();
        // if the matched KeyPoint alreadly produced a map point:
        // 1. relate this KeyPoint to the map point
        // 2. the map point add a observer about this KeyPoint
        // 3. break triangulating this keypoint
        if (mp_id >= 0) {
          image.setMapptOfKpt(i, mp_id);
          map->getMapPoint(mp_id).addObserver(image.id(), i);
          goto PRODUCE_NEXT_MAPPT;
        }  // else:
        cv::Mat1d proj2;
        cv::hconcat(image2.Rcw(), image2.tcw(), proj2);
        proj2 = sys.camera_K() * proj2;
        views.emplace_back(proj2);
        pt2s.emplace_back(kp2.pt());
      }  // finnish add related views
    }    // finish adding all tracked views and points
    {    // start to produce new map point...
      sp<I_Triangulate> tri(new MultiViewsTriangulater(views, pt2s));
      Point3 mp_pt;
      if (!tri->calculate(mp_pt)) continue;

      // insert the map point into global map
      tracked_image_and_kp.emplace_back(cur_image, i);
      auto mp = MapPoint::create(mp_pt, tracked_image_and_kp, db);
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
