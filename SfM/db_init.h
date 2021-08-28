//
/// \file src/model/image.h
///
/// \author zxm
/// \email xingmingzhangssr@gmail
/// \version 0.1
//

#ifndef __monocular_db_init_h__
#define __monocular_db_init_h__

class DB;
namespace fs = std::filesystem;

/// \class DBInit
/// \brief detect all keypoints in global images database, compute their match
/// results, and save the whole result in boost serialization binary files
/// \note
/// *
///
class DBInit {
  fs::path _images_dir, _out_dir;
  sp<DB> _db;

 public:
  /// \brief constructor
  /// \param images_dir the directory containing all images(no any other files)
  /// \param out_dir the directory output database files(images.binary,
  /// image_graph.binary...)
  DBInit(const fs::path &images_dir, const fs::path &out_dir);

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
