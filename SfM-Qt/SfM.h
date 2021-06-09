#pragma once

#include <QImage>
#include <QThread>
#include <any>

// use OptimizaSetting::OptMode enum...
#include "OptimizeSetting.h"

/**
 * @brief load all images in another thread for unblocking Ui-thread
 */
class LoadImagesThread : public QThread {
  Q_OBJECT

 public:
  LoadImagesThread(const std::string &path);
  void run() override;
 signals:
  void logMsg(const QString &msg);

 private:
  std::string _path;
};

class DB;
class Initialize;
class Map;
class IncrSfM;
struct VisualizationData;

/**
 * @brief definite state machine to execute the instructions
 */
class SfM : public QObject {
  Q_OBJECT

 signals:
  void log(const QString &);
  void visualizeData(const VisualizationData &);
  void showCurrentImage(int);

 public slots:
  void setDBPaths(const std::string &, const std::string &);
  void setInitImagePair(int, int);
  void setCameraConf(double fx, double fy, double cx, double cy);
  void setRuntimeConf(const std::any &);
  void setBAMode(OptimizeSetting::OptMode);

 public:
  /**
   * @brief all state of SfM system automatic-state-machine
   */
  enum class State {
    ORIGINAL,
    READY,
    INITIALIZING,
    INCR_SFM_READY,
    INCR_SFM_PNP_OK,
    INCR_SFM_MAP_OK,
    INCR_SFM_BA_OK,
    FAILED
  };

  State state() const;
  QString stateStr() const;
  bool isBusy() const;
  bool next();

  /**
   * @brief read all alternative image pairs form \a _db
   * @return std::vector<std::pair<int, int>>
   */
  std::vector<std::pair<int, int>> getAllInitImagePairs() const;

  /**
   * @brief read a image from \a _db
   * @param id the image's id
   * @return QImage
   */
  QImage getImageFromDB(int id) const;

 private:
  std::string _image_path, _image_graph_path;
  int _image_1 = -1, _image_2 = -1;
  bool _is_camera_set = false;
  OptimizeSetting::OptMode _BA_mode = OptimizeSetting::OptMode::local_none;

  template <typename T>
  using sp = std::shared_ptr<T>;
  sp<DB> _db;
  sp<Initialize> _initialize;
  sp<Map> _map;
  sp<IncrSfM> _incr_sfm;

  State _state = State::ORIGINAL;
  std::atomic_bool _is_busy = false;

  /**
   * @brief initialize the SfM system
   * @return return true if initialization succeeds
   */
  bool init();

  /**
   * @brief get global map for visualization
   * @return vector<Point3>
   */
  std::any getMapData() const;
  /**
   * @brief get the whole cameras' pose(R | t) for visualization
   * @return vector<pair<cv::Mat1f, cv::Mat1f>> [(Rwc, twc)...]
   */
  std::any getCamerasPoseData() const;

  /**
   * @brief get all images for next pass of registing image adding to
   * increasing SfM system
   * @return std::vector<int>
   * @note:
   * 1. this method only used in \a getNextImage(), you should not call this
   * normally.
   */
  std::vector<int> getNextRegistingImages() const;
  /**
   * @note only used in \a getNextImage() and \a continueOnePass()
   */
  std::vector<int> _registering_images;
  /**
   * @note only used in \a getNextImage() and \a continueOnePass()
   */
  int _registering_idx = -1;

  /**
   * @brief get next image to calculate pose using pnp algorithm
   * @return
   * -1 : no images any more\n
   * non -1 : normal case
   */
  int getNextImage();

  /**
   * @brief get current image processing
   * @return
   */
  int getCurImage() const;

  /**
   * @brief run pnp algorithm to calculate pose(Tcw) of image-\p id
   * @param[in] id
   * @return
   */
  bool pnp(int id);

  /**
   * @brief create new map points
   * @param id
   * @return
   */
  bool createMapPoints(int id);

  /**
   * @brief Bundle Adjustment
   */
  void BA();

  /**
   * @brief continue register a new image
   */
  void continueOnePass();

  /**
   * @brief finish the whole SfM rebuild work
   * @return True if all images processed, else return False
   */
  bool finish() const;
};
