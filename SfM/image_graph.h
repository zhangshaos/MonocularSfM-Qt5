//
/// \file src/model/image_graph.h
///
/// \author zxm
/// \email xingmingzhangssrgmail
/// \version 0.1
//

#ifndef __monocularsfm_imagegraph_h__
#define __monocularsfm_imagegraph_h__

#include <opencv2/core.hpp>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "common_type.h"

/// \class ImageGraph
/// \brief a graph representing \ref Image and their relationship
/// \note
/// *
/// *
class ImageGraph {
 public:
  template <typename Archive>
  void serialize(Archive& ar, const unsigned int ver);

  /// \brief the adjacent node of the graph
  struct AdjNode {
    int image_id = -1;
    int64_t matches = -1;
    std::unordered_map<int, int> dmatches;

    /// \brief default constructor, set \a image_id and \a matches -1
    AdjNode() : image_id(-1), matches(-1) {}

    /// \brief constructor
    /// \param _image_id connected image's ID
    /// \param _dmatches count of all keypoints matches
    AdjNode(int _image_id, const std::vector<cv::DMatch>& _dmatches)
        : image_id(_image_id), matches(_dmatches.size()) {
      fillDMatches(_dmatches);
    }

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int ver);

    struct Greater {
      bool operator()(const AdjNode& l, const AdjNode& r) const {
        return l.matches > r.matches;
      }
    };
    // struct Hash {
    //  size_t operator()(const AdjNode& n) const {
    //    return std::hash<int>()(n.image_id);
    //  }
    //};
    bool operator==(const AdjNode& o) const { return image_id == o.image_id; }

   private:
    void fillDMatches(const std::vector<cv::DMatch>& _dmatches);
  };

  ///// \brief   prioriry_queue of other image sorted by matches in descending
  /// order.
  // using AdjNodeSet =
  //    std::priority_queue<AdjNode, std::vector<AdjNode>, AdjNode::Greater>;

  using AdjNodeSet = std::unordered_map<int, AdjNode>;

 private:
  std::unordered_map<int, AdjNodeSet>
      _g;  ///< map image_id to prioriry_queue of other image sorted by matches
           ///< in descending order
  mutable smtx _mtx;
  friend class DB;

 public:
  /// \brief add a connection \p img1 <=> \p img2
  /// \param[in] img1
  /// \param[in] img2
  /// \param[in] matches
  /// \note:
  /// 1. can be used in multithread
  void addConnection(int img1, int img2,
                     const std::vector<cv::DMatch>& dmatches);

  /// \brief test there is a connection between \p img1 and \p img2
  /// \param img1
  /// \param img2
  /// \return
  bool existConnection(int img1, int img2) const;

  /// \brief return all connected images with \p image_id
  /// \param[in] image_id
  /// \return
  const AdjNodeSet& getAllConnected(int image_id) const;

  /**
   * @brief return all connected images sorted by matches with \p image_id
   * @param[in] image_id
   * @return
   */
  std::vector<int> getAsendingConnected(int image_id) const;

  /// \brief return the image in \class ImageGraph which have most connections
  ///  with the \p part images
  /// \param[in] part
  /// \return -1 means the no \p part edges in graph, other means correct
  /// node
  [[deprecated("Use getConnectedOfPart() instead.")]] int
  getMostConnectedOfPart(const std::unordered_set<int>& part) const;

  /// \brief return the images in \class ImageGraph which have connections
  ///  with the \p part images in ascending order
  /// \param[in] part
  /// \return
  std::vector<int> getConnectedOfPart(
      const std::unordered_set<int>& part) const;

  /// \brief return set of {image id, key point index} of \p part and the key
  /// point index is matched to \p image_id's \p kp_idx-th keypoint \param
  /// image_id \param kp_idx \param part \return
  std::vector<std::pair<int, int>> getAllTrackedKptsOfPart(
      int image_id, int kp_idx, const std::unordered_set<int>& part) const;

  /// \brief get the image_id pair which having the most matches among all image
  /// pairs
  /// \return
  /// \retval std::pair<int, int>
  [[deprecated("Use getAllMatchedPair() instead.")]] std::pair<int, int>
  getMostMatchesPair() const;

  /// \brief get the image_id pairs which ranked by matches-count in ascending
  /// order \return \retval std::vector<std::pair<int, int>>
  std::vector<std::pair<int, int>> getAllMatchedPair() const;
};

#endif  // !__monocularsfm_imagegraph_h__
