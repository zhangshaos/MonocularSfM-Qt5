#ifndef __monocular_db_h__
#define __monocular_db_h__

class Image;
class ImageGraph;

namespace fs = std::filesystem;

/// \class DBInit
/// \brief the database manipluate \class Image dataset and \class ImageGraph
/// object \note
/// *
class DB {
  std::unordered_map<int, Image> _images;  ///< map image_id to Image object
  smtx _mtx_images;
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
  void addImage(const Image &img);

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

  /**
   * @brief minimize the space of \a _images
   */
  void saveSpace();
};

#endif
