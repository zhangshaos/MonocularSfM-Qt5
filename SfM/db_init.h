//
/// \file src/model/image.h
///
/// \author zxm
/// \email xingmingzhangssr@gmail
/// \version 0.1
//

#ifndef __monocular_db_init_h__
#define __monocular_db_init_h__

#include <boost/format.hpp>
#include <filesystem>
#include <memory>
//#include <mutex>

#include "image.h"
#include "image_graph.h"

namespace fs = std::filesystem;

/// \class DBInit
/// \brief the database manipluate \class Image dataset and \class ImageGraph
/// object \note
/// *
class DB {
  std::unordered_map<int, Image> _images;  ///< map image_id to Image object
  std::mutex _mtx_images;
  ImageGraph _g;  ///< relationship of \a _images
 public:
  /// \brief create database from files
  /// \param[in] images_dir "images.binary" file location
  /// \param[in] graph_dir "image_graph.binary" file location
  /// \return database
  /// \retval std::shared_ptr<DB>
  static sp<DB> createDB(const fs::path &images_dir, const fs::path &graph_dir);

  /// \brief
  /// \param[in] img
  /// \note
  /// * could be used in case of multi-thread
  void addImage(const Image &img) {
    ulock<mtx> lock(_mtx_images);
    assert(img.id() >= 0);
    _images[img.id()] = img;
  }

  /// \brief
  /// \return
  /// \retval const std::unordered_map<int, Image> &
  const std::unordered_map<int, Image> &images() const { return _images; }

  /// \brief
  /// \return
  /// \retval std::unordered_map<int, Image> &
  std::unordered_map<int, Image> &images() { return _images; }

  /// \brief get the relationship reference of all images
  /// \return
  /// \retval \class ImageGraph reference
  ImageGraph &G() { return _g; }

  /// \brief get the relationship reference of all images
  /// \return
  /// \retval \class ImageGraph reference
  const ImageGraph &G() const { return _g; }

  /// \brief return the image connection relationship
  /// \return
  /// \note:
  /// * never use this in multi-thread environment.
  std::string debugString() const;
};

/// \class DBInit
/// \brief detect all keypoints in global images database, compute their match
/// results, and save the whole result in boost serialization binary files
/// \note
/// *
///
class DBInit {
  fs::path _images_dir, _out_dir;
  sp<DB> _db{new DB};

 public:
  /// \brief constructor
  /// \param images_dir the directory containing all images(no any other files)
  /// \param out_dir the directory output database files(images.binary,
  /// image_graph.binary...)
  DBInit(const fs::path &images_dir, const fs::path &out_dir)
      : _images_dir(images_dir), _out_dir(out_dir), _db(new DB) {
    bool ok = fs::exists(images_dir) && fs::exists(out_dir);
    auto s = (boost::wformat(L"directory %s and %s must be existed!") %
              images_dir.c_str() % out_dir.c_str())
                 .str();
    assert(ok && s.c_str());
  }

  /// \brief initialize DB
  void runInitializing();

  /// \brief
  /// \return return database
  /// \retval std::shared_ptr<DB>
  sp<DB> createDB() const { return _db; }

  /// \brief save database to \a _out_dir
  /// \param[out] images_dir "images.binary" file location
  /// \param[out] graph_dir "image_graph.binary" file location
  void saveDB(fs::path &images_dir, fs::path &graph_dir) const;
};

#endif  // !__monocular_db_init_h__
