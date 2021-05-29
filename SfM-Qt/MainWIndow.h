#pragma once

#include <QtWidgets/QMainWindow>
#include <memory>

#include "OptimizeSetting.h"
#include "CameraSetting.h"
#include "RuntimeSetting.h"
#include "DB_Shower.h"
#include "InitialPairSetting.h"

namespace Ui {
class MainWIndowClass;
}

class MainWIndow : public QMainWindow {
  Q_OBJECT

 public:
  MainWIndow(QWidget *parent = Q_NULLPTR);
  ~MainWIndow();

 private slots:
  void dataOpenFold();
  void dataOpenDB();
  void buildContinue();
  void buildOptimize();
  void buildInitPair();
  void confCamera();
  void confRuntime();

 signals:

 private:
  Ui::MainWIndowClass *ui;
  void initSignalSlot() const;
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
};
