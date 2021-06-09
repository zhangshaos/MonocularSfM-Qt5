#pragma once

#include <QObject>
#include <any>
#include <memory>
#include <mutex>

namespace pcl {
namespace visualization {
class PCLVisualizer;
}
template <typename Point>
class PointCloud;
class PointXYZ;
}  // namespace pcl
class QVTKOpenGLNativeWidget;
namespace cv {
template <typename T>
class Mat_;
using Mat1f = Mat_<float>;
template <typename T>
class Point3_;
using Point3d = Point3_<double>;
}  // namespace cv

struct VisualizationData {
  /**
   * @brief
   * Points: draw vector<Point3>, and the data will be not removed
   * until call another 2 draw-Points(except current call)\n
   * Cameras: draw vector<pair<cv::Mat1f(Rwc), cv::Mat1f(twc)>>, and the data
   * will be not removed until call another 2 draw-Cameras(except present
   * call)\n TempPoints: draw temporary points, which will be not removed until
   * call another draw-X action\n
   * TempCameras: draw temporary cameras, which will be not removed until
   * call another draw-X action\n
   * Update: change points color and removed the old cameras and points
   */
  enum {
    Points,
    Cameras,
    TempPoints,
    TempCameras,
    TempClear,
    Update
  } type = Update;
  std::any data;
  double r, g, b;
};

class PCLShower : public QObject {
  Q_OBJECT

 public slots:
  void visualizeSfM(const VisualizationData &data);

 public:
  PCLShower(QObject *parent);
  void setupUi(QVTKOpenGLNativeWidget *ui_widget);

  using Point3 = cv::Point3d;
  using Points_t = std::vector<Point3>;
  using CameraPose_t = std::vector<std::pair<cv::Mat1f, cv::Mat1f>>;

  static std::string generateNewID(const char preffix[]);

 private:
  /**
   * @brief draw global map points in \a _pcl_visualizer
   * @param r
   * @param g
   * @param b
   * @param temp
   */
  void drawPoints(double r, double g, double b, bool temp = false);
  /**
   * @brief draw all carmeras' poses in \a _pcl_visualizer
   * @param r
   * @param g
   * @param b
   * @param temp
   */
  void drawCamerasPose(double r, double g, double b, bool temp = false);

  /**
   * @brief change points color and removed the old cameras and points
   */
  void updateDraw();

  /**
   * @brief clear temporary drawed result
   */
  void clearTempDraw();

  // Main UI
  std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_visualizer;
  QVTKOpenGLNativeWidget *ui = nullptr;

  Points_t points_buf;
  CameraPose_t camera_poses_buf;
  std::string old_point_id, new_point_id;
  std::vector<std::string> old_camera_ids, new_camera_ids;
  std::string temp_point_id, temp_camera_id;
  std::mutex _mtx;  //< only used in unique export visualizationSfM()
};
