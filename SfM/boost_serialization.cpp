//
/// \file src/model/boost_serialization.cpp
/// Because I do not want to use define boost template serialize() function in
/// header file, so I use depart template instance here.
/// \author xingmingzhangssr@gmail
/// \version 0.1
//
//#include <boost/iostreams/filtering_stream.hpp>
//#include <boost/iostreams/filter/zstd.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/priority_queue.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/unordered_set.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/weak_ptr.hpp>

#include <opencv2/opencv.hpp>
#include "image.h"
#include "image_graph.h"

using BinIAr = boost::archive::binary_iarchive;
using BinOAr = boost::archive::binary_oarchive;

/// \brief serialize for \class opencv::Mat and \class opencv::DMatch
/// \note none
namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, cv::Mat& mat, const unsigned int) {
  int cols, rows, type;
  bool continuous;

  if (Archive::is_saving::value) {
    cols = mat.cols;
    rows = mat.rows;
    type = mat.type();
    continuous = mat.isContinuous();
  }

  ar& cols& rows& type& continuous;

  if (Archive::is_loading::value) mat.create(rows, cols, type);

  if (continuous) {
    int elem_sz = rows * cols;
    if (elem_sz == 0) return;
    const unsigned int data_size = elem_sz * mat.elemSize();
    ar& boost::serialization::make_array(mat.ptr(), data_size);
  } else {
    int elem_sz = cols;
    if (elem_sz == 0) return;
    const unsigned int row_size = elem_sz * mat.elemSize();
    for (int i = 0; i < rows; i++) {
      ar& boost::serialization::make_array(mat.ptr(i), row_size);
    }
  }
}

template <>
void serialize<BinIAr>(BinIAr& ar, cv::Mat& mat, const unsigned int) {
  int cols, rows, type;
  bool continuous;

  if (BinIAr::is_saving::value) {
    cols = mat.cols;
    rows = mat.rows;
    type = mat.type();
    continuous = mat.isContinuous();
  }

  ar& cols& rows& type& continuous;

  if (BinIAr::is_loading::value) mat.create(rows, cols, type);

  if (continuous) {
    int elem_sz = rows * cols;
    if (elem_sz == 0) return;
    const unsigned int data_size = elem_sz * mat.elemSize();
    ar& boost::serialization::make_array(mat.ptr(), data_size);
  } else {
    int elem_sz = cols;
    if (elem_sz == 0) return;
    const unsigned int row_size = elem_sz * mat.elemSize();
    for (int i = 0; i < rows; i++) {
      ar& boost::serialization::make_array(mat.ptr(i), row_size);
    }
  }
}

template <>
void serialize<BinOAr>(BinOAr& ar, cv::Mat& mat, const unsigned int) {
  int cols, rows, type;
  bool continuous;

  if (BinOAr::is_saving::value) {
    cols = mat.cols;
    rows = mat.rows;
    type = mat.type();
    continuous = mat.isContinuous();
  }

  ar& cols& rows& type& continuous;

  if (BinOAr::is_loading::value) mat.create(rows, cols, type);

  if (continuous) {
    int elem_sz = rows * cols;
    if (elem_sz == 0) return;
    const unsigned int data_size = elem_sz * mat.elemSize();
    ar& boost::serialization::make_array(mat.ptr(), data_size);
  } else {
    int elem_sz = cols;
    if (elem_sz == 0) return;
    const unsigned int row_size = elem_sz * mat.elemSize();
    for (int i = 0; i < rows; i++) {
      ar& boost::serialization::make_array(mat.ptr(i), row_size);
    }
  }
}

template <class Archive>
void serialize(Archive& ar, cv::DMatch& dm, const unsigned int) {
  ar& dm.distance& dm.imgIdx& dm.queryIdx& dm.trainIdx;
}

template <>
void serialize<BinIAr>(BinIAr& ar, cv::DMatch& dm, const unsigned int) {
  ar& dm.distance& dm.imgIdx& dm.queryIdx& dm.trainIdx;
}

template <>
void serialize<BinOAr>(BinOAr& ar, cv::DMatch& dm, const unsigned int) {
  ar& dm.distance& dm.imgIdx& dm.queryIdx& dm.trainIdx;
}

}  // namespace serialization
}  // namespace boost

/// \brief serialize for \class KeyPoint
/// \note none
template <typename Archive>
void KeyPoint::serialize(Archive& ar, const unsigned int ver) {
  ar& _mappoint_id;
  ar& _kp.angle& _kp.class_id& _kp.octave& _kp.pt.x& _kp.pt.y& _kp.response& _kp
      .size;
}
template <>
void KeyPoint::serialize<BinIAr>(BinIAr& ar, const unsigned int ver) {
  ar& _mappoint_id;
  ar& _kp.angle& _kp.class_id& _kp.octave& _kp.pt.x& _kp.pt.y& _kp.response& _kp
      .size;
}
template <>
void KeyPoint::serialize<BinOAr>(BinOAr& ar, const unsigned int ver) {
  ar& _mappoint_id;
  ar& _kp.angle& _kp.class_id& _kp.octave& _kp.pt.x& _kp.pt.y& _kp.response& _kp
      .size;
}

/// \brief serialize for \class Image
/// \note none
template <typename Archive>
void Image::serialize(Archive& ar, const unsigned int ver) {
  // assert(_gray_mat.type() == CV_8U);
  // assert(_rgb_mat.type() == CV_8UC3);
  ar & self->id;
  // ar& self->gray_mat& self->rgb_mat;
  ar & self->path;
  ar & self->kpts;
  ar & self->descp;
  ar & self->Rcw & self->tcw;
  if (Archive::is_loading::value) {
    assert(!self->path.empty());
    self->rgb_mat = cv::imread(self->path);
    assert(!self->rgb_mat.empty());
  }
}
template <>
void Image::serialize<BinIAr>(BinIAr& ar, const unsigned int ver) {
  // assert(_gray_mat.type() == CV_8U);
  // assert(_rgb_mat.type() == CV_8UC3);
  ar & self->id;
  // ar& self->gray_mat& self->rgb_mat;
  ar & self->path;
  ar & self->kpts;
  ar & self->descp;
  ar & self->Rcw & self->tcw;
  if (BinIAr::is_loading::value) {
    assert(!self->path.empty());
    self->rgb_mat = cv::imread(self->path);
    assert(!self->rgb_mat.empty());
  }
}
template <>
void Image::serialize<BinOAr>(BinOAr& ar, const unsigned int ver) {
  // assert(_gray_mat.type() == CV_8U);
  // assert(_rgb_mat.type() == CV_8UC3);
  ar & self->id;
  // ar& self->gray_mat& self->rgb_mat;
  ar & self->path;
  ar & self->kpts;
  ar & self->descp;
  ar & self->Rcw & self->tcw;
  if (BinOAr::is_loading::value) {
    assert(!self->path.empty());
    self->rgb_mat = cv::imread(self->path);
    assert(!self->rgb_mat.empty());
  }
}

/// \brief serialize for \class ImageGraph::AdjNode
/// \note none
template <typename Archive>
void ImageGraph::AdjNode::serialize(Archive& ar, const unsigned int ver) {
  ar& image_id& matches& dmatches;
}
template <>
void ImageGraph::AdjNode::serialize<BinIAr>(BinIAr& ar,
                                            const unsigned int ver) {
  ar& image_id& matches& dmatches;
}
template <>
void ImageGraph::AdjNode::serialize<BinOAr>(BinOAr& ar,
                                            const unsigned int ver) {
  ar& image_id& matches& dmatches;
}

/// \brief serialize for \class ImageGraph
/// \note none
template <typename Archive>
void ImageGraph::serialize(Archive& ar, const unsigned int ver) {
  ar& _g;
}
template <>
void ImageGraph::serialize<BinIAr>(BinIAr& ar, const unsigned int ver) {
  ar& _g;
}
template <>
void ImageGraph::serialize<BinOAr>(BinOAr& ar, const unsigned int ver) {
  ar& _g;
}