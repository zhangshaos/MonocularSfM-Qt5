#include "SfM.h"

#include <magic_enum.hpp>

#include "PCLShower.h"
#include "SFM/incr_sfm.h"
#include "SfM/db_init.h"
#include "SfM/initialize.h"
#include "SfM/system_info.h"

LoadImagesThread::LoadImagesThread(const std::string& path)
    : QThread(), _path(path) {}

void LoadImagesThread::run() {
  qDebug("loading images......\n");

  DBInit db_init(_path, "./");
  db_init.runInitializing();  // initializing

  auto db = db_init.createDB();  // get DB object
  QString msg =
      "************ Total images: %1 ************\n"
      "************ Image Graph Infos ************\n%2\n";
  emit logMsg(msg.arg(db->images().size())
                  .arg(QString::fromStdString(db->debugString())));

  fs::path images_path, graph_path;
  db_init.saveDB(images_path, graph_path);  // save DB to disk
  msg =
      "\n************ DB file saving path ************\n"
      "* images_path is \"%1\"\n* graph_path is \"%2\"\n"
      "*********************************************\n";
  emit logMsg(msg.arg(QString::fromStdString(images_path.string()))
                  .arg(QString::fromStdString(graph_path.string())));
}

void SfM::setDBPaths(const std::string& a, const std::string& b) {
  _image_graph_path = a;
  _image_path = b;
  qDebug(
      "\n************ DB file path ************\n"
      "* images path is %s,\n"
      "* image graph path is %s\n"
      "**************************************\n",
      _image_path.c_str(), _image_graph_path.c_str());
}

void SfM::setInitImagePair(int a, int b) {
  _image_1 = a;
  _image_2 = b;
  qDebug(
      "\n****** initial image pair id ******\n"
      "* image-1 id is %d,\n"
      "* image-2 id is %d\n"
      "***********************************\n",
      _image_1, _image_2);
}

void SfM::setCameraConf(const std::vector<double>& conf) {
  sys.camera_K(conf[0], conf[1], conf[2], conf[3]);
  sys.distort_parameter(conf[4], conf[5], conf[6], conf[7]);
  _is_camera_set = true;
  qDebug(
      "\n********* camera parameters *********\n"
      "fx = %f, fy = %f, cx = %f, cy = %f\n"
      "k1 = %f, k2 = %f, p1 = %f, p2 = %f\n"
      "*************************************\n",
      conf[0], conf[1], conf[2], conf[3], conf[4], conf[5], conf[6], conf[7]);
}

void SfM::setRuntimeConf(const std::any& _conf) {
  using namespace std;
  auto conf = make_shared<SystemInfo::SysConfig>(
      any_cast<SystemInfo::SysConfig>(_conf));
  sys.setConf(conf);
  qDebug(
      "\n********* runtime conf *********"
      "%s********************************\n",
      conf->toString().data());
}

void SfM::setBAMode(OptimizeSetting::OptMode m) {
  _BA_mode = m;
  qDebug(
      "\n************ BA ************\n"
      "BA mode is %s\n"
      "****************************\n",
      magic_enum::enum_name(m).data());
}

SfM::State SfM::state() const { return _state; }

QString SfM::stateStr() const {
  return QString(magic_enum::enum_name(_state).data());
}

bool SfM::isBusy() const { return _is_busy; }

bool SfM::next() {
  using namespace std;
  if (_is_busy.exchange(true)) {
    return false;
  }
  switch (_state) {
    case State::ORIGINAL: {
      if (_is_camera_set && !_image_path.empty() &&
          !_image_graph_path.empty()) {
        // load db
        _db = DB::createDB(_image_path, _image_graph_path);
        _initialize = make_shared<Initialize>(_db);
        _map = make_shared<Map>();
        _state = State::READY;

        emit log("【下一步】选择重建的初始化图片对");
      }
    } break;
    case State::READY: {
      if (_image_1 >= 0 && _image_2 >= 0) _state = State::INITIALIZING;

      emit log("【下一步】初始化");
    } break;
    case State::INITIALIZING: {
      if (init()) {
        _incr_sfm = make_shared<IncrSfM>(_db, _map, _initialize->used_images(),
                                         _initialize->init_image());
        try {
          VisualizationData cam_data;
          cam_data.type = VisualizationData::Cameras;
          cam_data.data = getCamerasPoseData();
          cam_data.r = 0.6, cam_data.g = 0., cam_data.b = 0.;
          emit visualizeData(cam_data);

          VisualizationData pts_data;
          pts_data.type = VisualizationData::Points;
          pts_data.data = getMapData();
          pts_data.r = 0.6, pts_data.g = 0., pts_data.b = 0.;
          emit visualizeData(pts_data);
        } catch (const bad_any_cast& e) {
          qDebug("*** Error *** SfM::next State::INITIALIZING any_cast error");
        } catch (const exception& e) {
          throw;
        }
        _state = State::INCR_SFM_READY;

        emit log("【下一步】加入新图片，并求解新图片位姿");
      } else {
        emit log("SfM 系统初始化失败");
      }
    } break;
    case State::INCR_SFM_READY: {
      int next = getNextImage();
      if (next == -1) {
        emit log("SfM Pnp 位姿求解全部失败");
        _state = State::FAILED;
      } else {
        emit showCurrentImage(next);

        if (pnp(next)) {
          {
            // show debug pose of camera
            VisualizationData data;
            data.type = VisualizationData::TempCameras;
            data.data = PCLShower::CameraPose_t({_incr_sfm->debugPose()});
            data.r = 0.6, data.g = 0., data.b = 0.;
            emit visualizeData(data);

            // show debug pnp points-tracking...
            VisualizationData data_pts;
            data_pts.type = VisualizationData::TempPoints;
            data_pts.data = PCLShower::Points_t(_incr_sfm->debugPoints());
            data_pts.r = 0.6, data_pts.g = 0., data_pts.b = 0.;
            emit visualizeData(data_pts);
          }

          VisualizationData data;
          data.type = VisualizationData::Cameras;
          data.data = getCamerasPoseData();
          data.r = 0., data.g = 0.6, data.b = 0.;
          emit visualizeData(data);
          _state = State::INCR_SFM_PNP_OK;

          emit log("【下一步】生成新的点云数据");
        }  // plus failed time of pnp
      }
    } break;
    case State::INCR_SFM_PNP_OK: {
      int cur = getCurImage();
      assert(cur > -1);
      if (createMapPoints(cur)) {
        VisualizationData temp_clear;
        temp_clear.type = VisualizationData::TempClear;
        emit visualizeData(temp_clear);

        VisualizationData data;
        data.type = VisualizationData::Points;
        data.data = getMapData();
        data.r = 0., data.g = 0.6, data.b = 0.;
        emit visualizeData(data);
        _state = State::INCR_SFM_MAP_OK;

        emit log("【下一步】准备 BA 优化");
      } else {
        emit log("SfM Map 地图点太少");
        _state = State::FAILED;
      }
    } break;
    case State::INCR_SFM_MAP_OK: {
      BA();
      try {
        VisualizationData cam_data;
        cam_data.type = VisualizationData::Cameras;
        cam_data.data = getCamerasPoseData();
        cam_data.r = 0., cam_data.g = 0.6, cam_data.b = 0.;
        emit visualizeData(cam_data);

        VisualizationData pts_data;
        pts_data.type = VisualizationData::Points;
        pts_data.data = getMapData();
        pts_data.r = 0., pts_data.g = 0.6, pts_data.b = 0.;
        emit visualizeData(pts_data);
      } catch (const bad_any_cast& e) {
        qDebug("*** Error *** SfM::next State::INCR_SFM_MAP_OK any_cast error");
      } catch (const exception& e) {
        throw;
      }
      _state = State::INCR_SFM_BA_OK;
    } break;
    case State::INCR_SFM_BA_OK: {
      continueOnePass();
      VisualizationData data;
      data.type = VisualizationData::Update;
      emit visualizeData(data);
      _state = State::INCR_SFM_READY;

      emit log("【下一步】加入新图片，并求解新图片位姿");
    } break;
    case State::FAILED: {
      // SfM rebuild is over!
      emit log("【重建失败】");

      _is_busy = false;
      return finish();
    } break;
    default:
      assert(0);
  }
  emit log(QString("当前系统状态: %1 \n").arg(stateStr()));
  _is_busy = false;
  return true;
}

std::vector<std::pair<int, int>> SfM::getAllInitImagePairs() const {
  return _db->G().getAllMatchedPair();
}

QImage SfM::getImageFromDB(int id) const {
  // auto m = _db->images().at(id).rgb_mat().clone();
  // cv::cvtColor(m, m, cv::COLOR_BGR2RGB);
  // auto image = QImage(static_cast<uchar*>(m.data), m.cols, m.rows,
  //                    m.step[0], QImage::Format_RGB888)
  //                 .copy();
  // return image;
  auto& m = _db->images().at(id).rgb_mat();
  auto t = QImage(static_cast<const uchar*>(m.data), m.cols, m.rows, m.step[0],
                  QImage::Format_RGB888)
               .rgbSwapped();
  return t;
}

bool SfM::init() { return _initialize->runInStep(_image_1, _image_2, _map); }

std::any SfM::getMapData() const { return _map->getAllPoints(); }

std::any SfM::getCamerasPoseData() const {
  using namespace std;
  vector<pair<cv::Mat1f, cv::Mat1f>> poses;
  for (int image_id : *_incr_sfm->used_images) {
    const Image& image = _db->images().at(image_id);
    cv::Mat1f Rwc, twc;
    cv::Mat1d(image.Rcw().t()).convertTo(Rwc, CV_32F);
    cv::Mat1d(-1 * image.tcw()).convertTo(twc, CV_32F);
    poses.emplace_back(Rwc, twc);
  }
  int id = getCurImage();
  // if id == -1, which means you not called getNextImages() before.
  // the reason usually is when you before State::INCR_SFM_READY
  if (id >= 0 && _incr_sfm->used_images->count(id) == 0) {
    const Image& image = _db->images().at(id);
    cv::Mat1f Rwc, twc;
    cv::Mat1d(image.Rcw().t()).convertTo(Rwc, CV_32F);
    cv::Mat1d(-image.tcw()).convertTo(twc, CV_32F);
    poses.emplace_back(Rwc, twc);
  }
  return poses;
}

std::vector<int> SfM::getNextRegistingImages() const {
  return _db->G().getConnectedOfPart(*_incr_sfm->used_images);
}

int SfM::getNextImage() {
  if (_registering_images.empty()) {
    _registering_images = getNextRegistingImages();
    _registering_idx = -1;
  }
  if (_registering_idx < static_cast<int>(_registering_images.size()) - 1)
    return _registering_images[++_registering_idx];
  return -1;
}

int SfM::getCurImage() const {
  if (_registering_idx >= 0 &&
      _registering_idx < static_cast<int>(_registering_images.size()))
    return _registering_images[_registering_idx];
  return -1;
}

bool SfM::pnp(int id) { return _incr_sfm->runPnp(id); }

bool SfM::createMapPoints(int id) { return _incr_sfm->runMapPoints(id); }

void SfM::BA() {
  int cur_image = getCurImage();
  switch (_BA_mode) {
    case OptimizeSetting::OptMode::local_none: {
      _incr_sfm->runBA(true, true, -1);  // or true, true, cur_image
    } break;
    case OptimizeSetting::OptMode::local_exclude_init_cur: {
      _incr_sfm->runBA(true, false, cur_image);
    } break;
    case OptimizeSetting::OptMode::local_all: {
      _incr_sfm->runBA(true, false, -1);
    } break;
    case OptimizeSetting::OptMode::global_none: {
      _incr_sfm->runBA(false, true, -1);  // or false, true, cur_image
    } break;
    case OptimizeSetting::OptMode::global_exclude_init_cur: {
      _incr_sfm->runBA(false, false, cur_image);
    } break;
    case OptimizeSetting::OptMode::global_all: {
      _incr_sfm->runBA(false, false, -1);
    } break;
    default:
      assert(0);
  }
}

void SfM::continueOnePass() {
  _registering_images.clear();
  _registering_idx = -1;
}

bool SfM::finish() const {
  return _db->images().size() == _incr_sfm->used_images->size();
}
