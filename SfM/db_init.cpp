
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/format.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <execution>
#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "global_config.h"
#include "image.h"
#include "image_graph.h"
#include "system_info.h"

// here
#include "db.h"
#include "db_init.h"
#include "kpt_extract_and_match.h"

using Format = boost::format;

DBInit::DBInit(const fs::path& images_dir, const fs::path& out_dir)
    : _images_dir(images_dir), _out_dir(out_dir), _db(new DB) {
  bool ok = fs::exists(images_dir) && fs::exists(out_dir);
  auto s = (boost::wformat(L"directory %s and %s must be existed!") %
            images_dir.c_str() % out_dir.c_str())
               .str();
  assert(ok && s.c_str());
}

void DBInit::runInitializing() {
  using namespace std;

  cout << "========================================\n"
          "====== INITIALIZING THE GLOBAL DB ======\n"
          "========================================\n";

  // set timer1
  constexpr char timer1_name[] = "extract keypoints from image";
  sys.startTimeRecord(timer1_name);

  // extraction keypoints and feature description for each picture
  vector<fs::path> files;
  for (auto& entry : fs::directory_iterator(_images_dir)) {
    auto ext =
        boost::algorithm::to_lower_copy(entry.path().extension().string());
    if (entry.is_regular_file() && (ext == ".jpg" || ext == ".png")) {
      files.emplace_back(entry.path());
    }
  }
  for_each(std::execution::par, files.begin(), files.end(),
           [this](const fs::path& f) {
             auto path = f.string();
             cv::Mat m = cv::imread(path);
             // assert(m.type() == CV_8UC3);
             sp<I_ExtractKeyPoints> extract(new SIFT_ExtractKeyPoints(m, path));
             _db->addImage(extract->run());
           });
  sys.stopTimeRecord(timer1_name);
  cout << sys.getTimeRecord(timer1_name) << endl;

  // set timer2
  constexpr char timer2_name[] = "match all images";
  sys.startTimeRecord(timer2_name);

  // compute the matched relationship of each image pair
  vector<pair<int, int>> image_pairs;
  auto& images = _db->images();
  for (auto& p1 : images) {
    int img1_id = p1.first;
    auto& img1 = p1.second;
    for (auto& p2 : images) {
      int img2_id = p2.first;
      auto& img2 = p2.second;
      if (img1_id < img2_id) image_pairs.emplace_back(img1_id, img2_id);
    }
  }
  // do matching...
  for_each(execution::par, image_pairs.begin(), image_pairs.end(),
           [this, &images](const pair<int, int>& pr) {
             const Image &img1 = images.at(pr.first),
                         &img2 = images.at(pr.second);
             // TODO: 实现抢占式匹配
             // 在正式开始特征点匹配前，测试两张图片的匹配度，如果匹配度低
             // 则直接跳过匹配，以此减少匹配时间！
             sp<I_MatchFeatures> match(new Flann_Matcher(
                 img1.kpts(), img1.descp(), img2.kpts(), img2.descp()));
             // DMatch
             vector<cv::DMatch> dms;
             if (match->run(dms) > sys.min_matches_edge_in_create_image_graph())
               _db->G().addConnection(pr.first, pr.second, dms);
           });
  sys.stopTimeRecord(timer2_name);
  cout << sys.getTimeRecord(timer2_name) << endl;
}

void DBInit::saveDB(fs::path& images_dir, fs::path& graph_dir) const {
  using namespace std;
  // set timer
  constexpr char timer_name[] = "save DB to files";
  sys.startTimeRecord(timer_name);

  (images_dir = _out_dir) /= ("images.binary");
  (graph_dir = _out_dir) /= ("image_graph.binary");
  images_dir = fs::absolute(images_dir);
  graph_dir = fs::absolute(graph_dir);

  ofstream f1(images_dir, ios_base::binary), f2(graph_dir, ios_base::binary);
  assert(f1.is_open() && f2.is_open());
  boost::archive::binary_oarchive bf1(f1), bf2(f2);
  _db->saveSpace();  // TODO 需要测试
  bf1 << _db->images();
  bf2 << _db->G();

  sys.stopTimeRecord(timer_name);
  cout << sys.getTimeRecord(timer_name) << endl;
}
