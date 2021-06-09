#pragma once

#include <QtWidgets/QMainWindow>
#include <any>
#include <future>
#include <memory>

#include "OptimizeSetting.h"
#include "PCLShower.h"

namespace Ui {
class MainWIndowClass;
}
struct VisualizationData;
class SfM;
class RuntimeSetting;
class InitialPairSetting;
class DB_Shower;
class CameraSetting;

class MainWIndow : public QMainWindow {
  Q_OBJECT

 public:
  MainWIndow(QWidget *parent = Q_NULLPTR);
  ~MainWIndow();

 public slots:
  void printLog(const QString &text);

 private slots:
  void dataOpenFold();
  void dataOpenDB();
  void buildContinue();
  void buildOptimize();
  void buildInitPair();
  void confCamera();
  void confRuntime();

  /**
   * @brief read image from \a _sfm to \a _init_pair_set for showing
   * @param image
   */
  void showInitImageFromDB(int image);

  void chooseLogFont();

  /**
   * @brief read image from \a _sfm and show it
   * @param image
   */
  void showCurrentImage(int image);

 signals:
  void pathOfDB(std::string, std::string);
  void initImagePair(int, int);
  void BAMode(OptimizeSetting::OptMode);

 private:
  /**
   * @brief main Ui
   */
  Ui::MainWIndowClass *ui;
  PCLShower _pcl;
  void initSignalSlot();

  /**
   * @brief main Structure from Motion system
   */
  std::shared_ptr<SfM> _sfm;

  std::future<bool> _sfm_result;

  /**
   * @brief 打开 BA 优化设置
   */
  std::shared_ptr<OptimizeSetting> _optimize_set;
  /**
   * @brief 打开相机参数设置
   */
  std::shared_ptr<CameraSetting> _camara_set;
  /**
   * @brief 打开系统运行设置
   */
  std::shared_ptr<RuntimeSetting> _runtime_set;
  /**
   * @brief 打开初始图片对设置
   */
  std::shared_ptr<InitialPairSetting> _init_pair_set;
  /**
   * @brief 打开 DB 可视化窗口
   */
  std::shared_ptr<DB_Shower> _db_shower;


  /**
   * @brief the images stored for visualization
   */
  std::unordered_map<int, QPixmap> _images;
};
