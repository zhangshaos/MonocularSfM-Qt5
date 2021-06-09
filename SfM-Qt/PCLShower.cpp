#include "PCLShower.h"

#include <opencv2/core.hpp>

#include "QVTKOpenGLNativeWidget.h"
#include "SfM.h"
#include "SfM/common_type.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include "vtkRenderWindow.h"

PCLShower::PCLShower(QObject* parent) : QObject(parent) {
  qRegisterMetaType<VisualizationData>("VisualizationData");
}

void PCLShower::setupUi(QVTKOpenGLNativeWidget* ui_widget) {
  assert(ui == nullptr && ui_widget != nullptr);
  ui = ui_widget;
  using namespace pcl;
  // QVTKOpenGL[Native]Widget use GenericOpenGLRenderWindow rather than Win32
  // window so you need to set up render window by yourself
  auto render = vtkSmartPointer<vtkRenderer>::New();
  auto opengl_win = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  opengl_win->AddRenderer(render);
  pcl_visualizer.reset(new pcl::visualization::PCLVisualizer(
      render, opengl_win, "pcl visualizer", false));
  ui->SetRenderWindow(pcl_visualizer->getRenderWindow());
  pcl_visualizer->setupInteractor(ui->GetInteractor(), ui->GetRenderWindow());
  pcl_visualizer->initCameraParameters();
  pcl_visualizer->setBackgroundColor(0.9, 0.9, 0.9);
  ui->update();

  // 测试数据
  while (0) {
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    auto GenRandomPoints = [] {
      constexpr int _MAX_ = 100;
      float x = rand() % _MAX_, y = rand() % _MAX_, z = rand() % _MAX_;
      return std::make_tuple(x, y, z);
    };
    auto GenRandomColor = [] {
      constexpr int _MAX_ = 256;
      uint8_t r = rand() % _MAX_, g = rand() % _MAX_, b = rand() % _MAX_;
      return std::make_tuple(r, g, b);
    };
    for (int i = 0; i < 1000; ++i) {
      float x, y, z;
      std::tie(x, y, z) = GenRandomPoints();
      uint8_t r, g, b;
      std::tie(r, g, b) = GenRandomColor();
      cloud->emplace_back(x, y, z, r, g, b);
    }  // 数据准备完毕
    pcl_visualizer->addPointCloud(cloud, "pts0");
    pcl_visualizer->setPointCloudRenderingProperties(
        visualization::PCL_VISUALIZER_POINT_SIZE, 4, "pts0");
    pcl_visualizer->addCoordinateSystem();
    // 测试相机数据
    while (0) {
      using namespace std;
      vector<PointXYZ> points{
          {0, 0, 0}, {5, 5, 4}, {-5, 5, 4}, {-5, -5, 4}, {5, -5, 4}};
      PointCloud<PointXYZ>::Ptr camera_xyzes(new PointCloud<PointXYZ>);
      // draw 0 4 3 2 1 4, 1 0 2 3 0
      camera_xyzes->emplace_back(points[0]);
      camera_xyzes->emplace_back(points[4]);
      camera_xyzes->emplace_back(points[1]);
      camera_xyzes->emplace_back(points[3]);
      camera_xyzes->emplace_back(points[2]);
      camera_xyzes->emplace_back(points[0]);

      camera_xyzes->emplace_back(points[1]);
      camera_xyzes->emplace_back(points[2]);
      camera_xyzes->emplace_back(points[4]);
      camera_xyzes->emplace_back(points[3]);
      camera_xyzes->emplace_back(points[0]);

      pcl_visualizer->addPolygon<PointXYZ>(camera_xyzes, 0.8, 0., 0.);

      cv::Mat1f camera_pos(3, 1);
      camera_pos << 50.f, 50.f, -20.f;
      pcl_visualizer->removeShape("polygon");
      Eigen::Matrix4f trans;
      trans << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -25, 0, 0, 0, 1;
      pcl_visualizer->updateShapePose("polygon", Eigen::Affine3f(trans));
      pcl_visualizer->setShapeRenderingProperties(
          visualization::PCL_VISUALIZER_COLOR, 0., 1., 0., "polygon");
    }
    ui->update();
  }
}

/**
 * @brief construct a camera polygon for pcl visualization
 * @param Rwc
 * @param twc
 * @param focal the half length of camera's width(height)
 * @return std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
 */
inline std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> Camera(
    const cv::Mat1f& Rwc, const cv::Mat1f& twc, float focal = 1) {
  using namespace std;
  using namespace pcl;
  using Pt3 = cv::Point3f;
  using Mat3 = cv::Matx31f;

  PointCloud<PointXYZ>::Ptr camera_xyzes(new PointCloud<PointXYZ>);

  try {
    /****************************************
     * fix bug:
     ****************************************
     * Twc != | Rcw^-1 -tcw |
     *        |   0      1  |
     ****************************************
     * | Rcw tcw |   | Rcw^-1 -Rcw^-1 * tcw |
     * |  0   1  | * |   0            1     |
     * 
     * = | E 0 | = E!!!
     *   | 0 1 |
     */
    cv::Mat1f _twc = Rwc * twc;
    Pt3 centr0 = _twc,
        centr1 = cv::Mat1f(Rwc * Mat3(focal, focal, focal) + _twc),
        centr2 = cv::Mat1f(Rwc * Mat3(-focal, focal, focal) + _twc),
        centr3 = cv::Mat1f(Rwc * Mat3(-focal, -focal, focal) + _twc),
        centr4 = cv::Mat1f(Rwc * Mat3(focal, -focal, focal) + _twc);

    vector<PointXYZ> points{
        {centr0.x, centr0.y, centr0.z}, {centr1.x, centr1.y, centr1.z},
        {centr2.x, centr2.y, centr2.z}, {centr3.x, centr3.y, centr3.z},
        {centr4.x, centr4.y, centr4.z},
    };

    // draw 0 4 3 2 1 4, 1 0 2 3 0
    camera_xyzes->emplace_back(points[0]);
    camera_xyzes->emplace_back(points[4]);
    camera_xyzes->emplace_back(points[1]);
    camera_xyzes->emplace_back(points[3]);
    camera_xyzes->emplace_back(points[2]);
    camera_xyzes->emplace_back(points[0]);

    camera_xyzes->emplace_back(points[1]);
    camera_xyzes->emplace_back(points[2]);
    camera_xyzes->emplace_back(points[4]);
    camera_xyzes->emplace_back(points[3]);
    camera_xyzes->emplace_back(points[0]);
  } catch (const cv::Exception& e) {
    cout << e.what() << endl;
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
  }

  return camera_xyzes;
}

void PCLShower::visualizeSfM(const VisualizationData& data) {
  using namespace std;
  using namespace pcl;
  unique_lock<mutex> lock(_mtx);
  switch (data.type) {
    case VisualizationData::TempClear: {
      clearTempDraw();
    } break;
    case VisualizationData::Update: {
      // remove old point and camera
      // and swap the double-buffer
      updateDraw();
    } break;
    case VisualizationData::TempPoints:
    case VisualizationData::Points: {
      points_buf = move(any_cast<Points_t>(data.data));
      drawPoints(data.r, data.g, data.b,
                 data.type == VisualizationData::TempPoints);
    } break;
    case VisualizationData::TempCameras:
    case VisualizationData::Cameras: {
      camera_poses_buf = move(any_cast<CameraPose_t>(data.data));
      drawCamerasPose(data.r, data.g, data.b,
                      data.type == VisualizationData::TempCameras);
    } break;
    default:
      assert(0);
  }

  ui->GetRenderWindow()->Render();
}

void PCLShower::drawPoints(double r, double g, double b, bool is_temp) {
  using namespace std;
  using namespace pcl;

  auto pts = std::make_shared<PointCloud<PointXYZ>>();
  for (auto& p : points_buf) {
    pts->emplace_back(p.x, p.y, p.z);
  }  // new pcl point cloud data

  // add new pcl point cloud
  auto new_id = generateNewID("pts");
  bool yes = pcl_visualizer->addPointCloud(pts, new_id);
  assert(yes);

  if (is_temp) {
    // remove old
    if (!temp_point_id.empty()) {
      pcl_visualizer->removePointCloud(temp_point_id);
    }
    swap(temp_point_id, new_id);

    // set temp point color
    pcl_visualizer->setPointCloudRenderingProperties(
        visualization::PCL_VISUALIZER_POINT_SIZE, 3, temp_point_id);
    pcl_visualizer->setPointCloudRenderingProperties(
        visualization::PCL_VISUALIZER_COLOR, r, g, b, temp_point_id);
  } else {
    swap(new_point_id, old_point_id);
    swap(new_id, new_point_id);

    // change ponts cloud color
    pcl_visualizer->setPointCloudRenderingProperties(
        visualization::PCL_VISUALIZER_POINT_SIZE, 3, new_point_id);
    pcl_visualizer->setPointCloudRenderingProperties(
        visualization::PCL_VISUALIZER_COLOR, r, g, b, new_point_id);
    // pcl_visualizer->setPointCloudRenderingProperties(
    //    visualization::PCL_VISUALIZER_POINT_SIZE, 5, old_point_id);
    if (!old_point_id.empty()) {
      pcl_visualizer->setPointCloudRenderingProperties(
          visualization::PCL_VISUALIZER_COLOR, 0., 0., 0., old_point_id);
    }

    // remove old pcl point cloud
    auto& old_id = new_id;
    if (!old_id.empty()) {
      yes = pcl_visualizer->removePointCloud(old_id);
      assert(yes);
    }
  }

  ui->update();
}

void PCLShower::drawCamerasPose(double r, double g, double b, bool is_temp) {
  using namespace std;
  using namespace pcl;

  vector<shared_ptr<PointCloud<PointXYZ>>> cameras;
  for (auto& pose : camera_poses_buf) {
    cameras.emplace_back(Camera(pose.first, pose.second, 5));
  }  // new cameras' point clouds

  // add new camera
  vector<string> cam_names;
  for (const auto& cam : cameras) {
    cam_names.emplace_back(generateNewID("cam"));
    bool existed = pcl_visualizer->addPolygon<PointXYZ>(cam, cam_names.back());
    assert(!existed);
  }

  if (is_temp) {
    // remove old
    if (!temp_camera_id.empty()) {
      pcl_visualizer->removeShape(temp_camera_id);
    }
    temp_camera_id = cam_names.front();
    // set color
    pcl_visualizer->setShapeRenderingProperties(
        visualization::PCL_VISUALIZER_COLOR, r, g, b, temp_camera_id);

  } else {
    swap(new_camera_ids, old_camera_ids);
    swap(cam_names, new_camera_ids);
    // change color
    for (const auto& cam : new_camera_ids) {
      pcl_visualizer->setShapeRenderingProperties(
          visualization::PCL_VISUALIZER_COLOR, r, g, b, cam);
    }
    for (const auto& cam : old_camera_ids) {
      pcl_visualizer->setShapeRenderingProperties(
          visualization::PCL_VISUALIZER_COLOR, 0., 0., 0., cam);
    }

    // remove old camera
    auto& old_cam_names = cam_names;
    for (const auto& cam : old_cam_names) {
      pcl_visualizer->removeShape(cam);
    }
  }

  ui->update();
}

std::string PCLShower::generateNewID(const char preffix[]) {
  static uint64_t id = 0;
  std::string str_id(preffix);
  str_id += std::to_string(id);
  id = id < std::numeric_limits<uint64_t>::max() ? id + 1 : 0;
  return str_id;
}

void PCLShower::updateDraw() {
  using namespace pcl;
  clearTempDraw();
  pcl_visualizer->removePointCloud(old_point_id);
  old_point_id = move(new_point_id);
  // set old point black
  pcl_visualizer->setPointCloudRenderingProperties(
      visualization::PCL_VISUALIZER_COLOR, 0., 0., 0., old_point_id);

  for (const auto& cam : old_camera_ids) {
    pcl_visualizer->removeShape(cam);
  }
  old_camera_ids = move(new_camera_ids);
  // set old cameras black
  for (const auto& cam : old_camera_ids) {
    pcl_visualizer->setShapeRenderingProperties(
        visualization::PCL_VISUALIZER_COLOR, 0., 0., 0., cam);
  }
}

void PCLShower::clearTempDraw() {
  if (!temp_point_id.empty()) {
    pcl_visualizer->removePointCloud(temp_point_id);
  }
  if (!temp_camera_id.empty()) {
    pcl_visualizer->removeShape(temp_camera_id);
  }
}
