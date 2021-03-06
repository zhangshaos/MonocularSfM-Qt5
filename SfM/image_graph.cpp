#include "image_graph.h"

#include <algorithm>
#include <atomic>
#include <execution>
#include <map>

#include "global_config.h"

void ImageGraph::addConnection(int src_image_id, int target,
                               const std::vector<cv::DMatch>& dmatches) {
  ulock<smtx> lock(_mtx);
  auto& q = _g[src_image_id];
  q[target] = AdjNode(target, dmatches);
  lock.unlock();

  std::vector<cv::DMatch> dmatches2 = dmatches;
  std::for_each(std::execution::par, dmatches2.begin(), dmatches2.end(),
                [](cv::DMatch& dm) { std::swap(dm.queryIdx, dm.trainIdx); });

  lock.lock();
  auto& q2 = _g[target];
  q2[src_image_id] = AdjNode(src_image_id, dmatches2);
  lock.unlock();
}

bool ImageGraph::existConnection(int img1, int img2) const {
  if (_g.count(img1) && _g.at(img1).count(img2)) {
    assert(_g.count(img2) && _g.at(img2).count(img1));
    return true;
  } else if (_g.count(img2) && _g.at(img2).count(img1)) {
    assert(_g.count(img1) && _g.at(img1).count(img2));
    return true;
  } else {
    return false;
  }
}

const ImageGraph::AdjNodeSet& ImageGraph::getAllConnected(int image_id) const {
  return _g.at(image_id);
}

std::vector<int> ImageGraph::getAsendingConnected(int image_id) const {
  using namespace std;
  auto& connected = getAllConnected(image_id);
  using IdMatches_t = pair<int, int64_t>;
  vector<IdMatches_t> id_matches;
  transform(connected.begin(), connected.end(),
            inserter(id_matches, id_matches.begin()),
            [](const AdjNodeSet::value_type& v) {
              return IdMatches_t{v.first, v.second.matches};
            });
  sort(id_matches.begin(), id_matches.end(),
       [](const IdMatches_t& a, const IdMatches_t& b) {
         return a.second > b.second;
       });
  vector<int> ans;
  transform(id_matches.begin(), id_matches.end(), inserter(ans, ans.begin()),
            [](const IdMatches_t& v) { return v.first; });
  return ans;
}

inline bool image_to_edges_less(const std::pair<int, int>& a,
                                const std::pair<int, int>& b) {
  return a.second < b.second;
}
int ImageGraph::getMostConnectedOfPart(
    const std::unordered_set<int>& part) const {
  using namespace std;
  // collect all nodes which conected with part but excluded with part. name the
  // collection S.
  // find the node in S which have most edges with part and return it.
  unordered_map<int, int> image_to_edges;
  for (auto n : part) {
    auto& adjnodes = getAllConnected(n);
    for (auto& node : adjnodes) {
      int id = node.first;
      if (!part.count(id)) ++image_to_edges[id];
    }
  }
  if (image_to_edges.empty()) return -1;
  auto image_edges = std::max_element(
      image_to_edges.begin(), image_to_edges.end(), image_to_edges_less);
  return image_edges->first;
}

std::vector<int> ImageGraph::getConnectedOfPart(
    const std::unordered_set<int>& part) const {
  using namespace std;
  unordered_map<int, int> image_to_edges;
  // ??????????????? part ??????????????????????????? part ???????????????????????????????????????
  for (auto n : part) {
    auto& adjnodes = getAllConnected(n);
    for (auto& node : adjnodes) {
      int id = node.first;
      if (!part.count(id)) ++image_to_edges[id];
    }
  }
  using ImageEdges = pair<int, int>;
  vector<ImageEdges> image_edges(image_to_edges.begin(), image_to_edges.end());
  // ???????????????????????????
  sort(image_edges.begin(), image_edges.end(),
       [](const ImageEdges& a, const ImageEdges& b) {
         return a.second > b.second;
       });  // in ascending order
  vector<int> images;
  transform(image_edges.begin(), image_edges.end(),
            inserter(images, images.begin()),
            [](const ImageEdges& t) -> int { return t.first; });
  return images;
}

std::vector<std::pair<int, int>> ImageGraph::getAllTrackedKptsOfPart(
    int image_id, int kp_idx, const std::unordered_set<int>& part) const {
  std::vector<std::pair<int, int>> res;
  auto& connected = getAllConnected(image_id);
  for (auto& p : connected) {
    int target_image = p.first;
    auto& match_res = p.second;
    if (!part.count(target_image)) continue;
    if (!match_res.dmatches.count(kp_idx)) continue;
    int target_kp_idx = match_res.dmatches.at(kp_idx);
    res.emplace_back(target_image, target_kp_idx);
  }
  return res;
}

std::pair<int, int> ImageGraph::getMostMatchesPair() const {
  using namespace std;
  smtx mtx;
  int most_matches = 0;
  int pr1 = -1, pr2 = -1;
  for_each(execution::par, _g.begin(), _g.end(),
           [&](const pair<int, AdjNodeSet>& src_node) {
             for_each(
                 execution::par, src_node.second.begin(), src_node.second.end(),
                 [&](const AdjNodeSet::value_type& node) {
                   int matches = (int)node.second.matches;
                   ulock<smtx> lock(mtx);
                   if (src_node.first < node.first && matches > most_matches) {
                     // cause of usage of \class ImageGraph(graph own two same
                     // edges <i,j> and <j,i>). So i use i<j to take only one
                     // pass
                     pr1 = src_node.first;
                     pr2 = node.first;
                     most_matches = matches;
                   }
                 });
           });
  return {pr1, pr2};
}

std::vector<std::pair<int, int>> ImageGraph::getAllMatchedPair() const {
  using namespace std;
  using Tuple = std::tuple<int, int, int64_t>;  // <image-id1, image-id2,
                                                // matched-keypoints>
  using Pair = pair<int, int>;                  // <image-id, matched-images>
  std::vector<Tuple> ranked;
  for (auto& p1 : _g) {
    int src_id = p1.first;
    auto& src = p1.second;
    for (auto& p2 : src) {
      int dest_id = p2.first;
      auto& dest = p2.second;
      if (src_id < dest_id) ranked.emplace_back(src_id, dest_id, dest.matches);
    }
  }
  sort(ranked.begin(), ranked.end(), [](const Tuple& t1, const Tuple& t2) {
    return get<2>(t1) > get<2>(t2);
  });
  vector<Pair> ans;
  for (auto& t : ranked) ans.emplace_back(get<0>(t), get<1>(t));
  return ans;
}

void ImageGraph::getMatchedKeyPoints(int img1, std::vector<int>& kps1, int img2,
                                     std::vector<int>& kps2) const {
  auto& dms = _g.at(img1).at(img2).dmatches;
  kps1.clear();
  kps2.clear();
  for (auto& dm : dms) {
    int i = dm.first, j = dm.second;
    kps1.emplace_back(i);
    kps2.emplace_back(j);
  }
}

void ImageGraph::AdjNode::fillDMatches(
    const std::vector<cv::DMatch>& _dmatches) {
  using namespace std;
  for_each(_dmatches.begin(), _dmatches.end(), [this](const cv::DMatch& dm) {
    dmatches[dm.queryIdx] = dm.trainIdx;
  });
  assert(dmatches.size() == _dmatches.size());
}
