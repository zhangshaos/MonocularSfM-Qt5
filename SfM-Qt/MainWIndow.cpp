#include "MainWIndow.h"

#include <vtkRenderWindow.h>

#include <QFileDialog>
#include <QFontDialog>
#include <QMessageBox>
#include <opencv2/core.hpp>

#include "CameraSetting.h"
#include "DB_Shower.h"
#include "InitialPairSetting.h"
#include "RuntimeSetting.h"
#include "SfM.h"
#include "ui_MainWIndow.h"

MainWIndow::MainWIndow(QWidget* parent)
    : QMainWindow(parent),
      ui(new Ui::MainWIndowClass),
      _sfm(new SfM),
      _optimize_set(new OptimizeSetting(this)),
      _camara_set(new CameraSetting(this)),
      _runtime_set(new RuntimeSetting(this)),
      _init_pair_set(new InitialPairSetting(this)),
      _db_shower(new DB_Shower(nullptr /*independent window*/)),
      _pcl(this) {
  ui->setupUi(this);
  _pcl.setupUi(ui->cloud_show_window);
  initSignalSlot();  // defined in MainWindowPCLShower.cpp
}

void MainWIndow::initSignalSlot() {
  connect(ui->action_choose_pair, SIGNAL(triggered()), this,
          SLOT(buildInitPair()));
  connect(ui->action_camera_conf, SIGNAL(triggered()), this,
          SLOT(confCamera()));
  connect(ui->action_run_conf, SIGNAL(triggered()), this, SLOT(confRuntime()));

  // set
  connect(this, &MainWIndow::pathOfDB, _sfm.get(), &SfM::setDBPaths);
  connect(this, &MainWIndow::initImagePair, _sfm.get(), &SfM::setInitImagePair);
  connect(_sfm.get(), &SfM::log, this, &MainWIndow::printLog);
  connect(_camara_set.get(), &CameraSetting::conf, _sfm.get(),
          &SfM::setCameraConf);
  connect(_runtime_set.get(), &RuntimeSetting::conf, _sfm.get(),
          &SfM::setRuntimeConf);
  connect(_init_pair_set.get(), &InitialPairSetting::image, this,
          &MainWIndow::showInitImageFromDB);
  connect(_init_pair_set.get(), &InitialPairSetting::imagePair, this,
          &MainWIndow::initImagePair);
  connect(this, &MainWIndow::BAMode, _sfm.get(), &SfM::setBAMode);
  connect(ui->choose_font, &QPushButton::clicked, this,
          &MainWIndow::chooseLogFont);
  connect(_sfm.get(), &SfM::visualizeData, &_pcl, &PCLShower::visualizeSfM,
          Qt::QueuedConnection);
  connect(_sfm.get(), &SfM::showCurrentImage, this,
          &MainWIndow::showCurrentImage);
}

MainWIndow::~MainWIndow() { delete ui; }

void MainWIndow::printLog(const QString& text) { ui->logs->append(text); }

void MainWIndow::dataOpenFold() {
  qDebug("action open fold is clicked\n");
  QFileDialog dialog(this, "打开图片文件夹");
  dialog.setFileMode(QFileDialog::Directory);
  if (dialog.exec()) {
    auto d = dialog.directory();
    qDebug("selected path is %s,\nabsolute path is %s,\ncanonical path is %s\n",
           qPrintable(d.path()), qPrintable(d.absolutePath()),
           qPrintable(d.canonicalPath()));
    // load images in another thread
    auto load_images_thread = new LoadImagesThread(d.path().toStdString());
    connect(load_images_thread, &LoadImagesThread::logMsg, this,
            &MainWIndow::printLog);
    connect(load_images_thread, &LoadImagesThread::finished, load_images_thread,
            &LoadImagesThread::deleteLater);
    load_images_thread->start();
  } else {
    qDebug("choose nothing...\n");
  }
}

void MainWIndow::dataOpenDB() {
  qDebug("action open DB is clicked\n");
  QFileDialog dialog(this,
                     "打开图片数据库和图片关系数据（按 CTRL 键进行多选）");
  dialog.setFileMode(QFileDialog::ExistingFiles);
  dialog.setNameFilter(
      "Image file and Image Graph file (images.binary image_graph.binary)");
  if (dialog.exec()) {
    auto names = dialog.selectedFiles();
    if (names.size() != 2) {
      QMessageBox::warning(
          this, "警告",
          "必须选择两个文件：一个为图片数据库，一个为图片关系数据");
      return;
    }
    qDebug("file names are:\n%s\n%s\n", qPrintable(names.at(0)),
           qPrintable(names.at(1)));
    if (names.at(0).contains("graph")) {
      emit pathOfDB(names.at(0).toStdString(), names.at(1).toStdString());
    } else if (names.at(1).contains("graph")) {
      emit pathOfDB(names.at(1).toStdString(), names.at(0).toStdString());
    } else {
      QMessageBox::warning(
          this, "警告",
          "必须选择两个文件：一个为图片数据库，一个为图片关系数据");
    }
  } else {
    qDebug("choose nothing...\n");
  }
}

void MainWIndow::buildContinue() {
  qDebug("action continue build is clicked\n");
  using namespace std;
  // check last SfM error
  if (_sfm_result.valid()) {
    try {
      _sfm_result.get();
    } catch (const std::exception& e) {
      QMessageBox::warning(this, "警告",
                           QString("上一轮重建发生错误：%1").arg(e.what()));
    }
  }
  // run SfM algorithm in another thread
  if (_sfm->isBusy()) return;
  _sfm_result = std::async(std::launch::async, [this] {
    // note: 因为 this 指针在此函数调用期间一定存活，
    // 因此直接使用 this._sfm.xxx 没有问题
    return _sfm->next();
  });
  // bind(&SfM::next, _sfm.get());
}

void MainWIndow::buildOptimize() {
  qDebug("action set optimize is clicked\n");
  if (_optimize_set->exec()) {
    auto m = _optimize_set->mode();
    emit BAMode(m);
  }
}

void MainWIndow::buildInitPair() {
  qDebug("action %s is clicked\n",
         qPrintable(ui->action_choose_pair->objectName()));
  if (_sfm->state() < SfM::State::READY) {
    printLog(
        "你必须先执行以下流程：\n"
        "[选择数据库] and [设置相机参数] => [继续重建] => [选择初始化图片对]");
    return;
  }
  // read all alternative choices about initial image pair
  auto pairs = _sfm->getAllInitImagePairs();
  _init_pair_set->setInitImagePairs(pairs);
  if (_init_pair_set->exec()) {
    // none
  }
}

void MainWIndow::confCamera() {
  qDebug("action %s is clicked\n",
         qPrintable(ui->action_camera_conf->objectName()));
  if (_camara_set->exec()) {
    auto [fx, fy, cx, cy] = _camara_set->getConf();
    emit _camara_set->conf(fx, fy, cx, cy);
  }
}

void MainWIndow::confRuntime() {
  qDebug("action %s is clicked\n",
         qPrintable(ui->action_run_conf->objectName()));
  if (_runtime_set->exec()) {
    auto conf = _runtime_set->getConf();
    emit _runtime_set->conf(conf);
  }
}

void MainWIndow::showInitImageFromDB(int id) {
  auto image = _sfm->getImageFromDB(id);
  _init_pair_set->addImage(id, image);
}

void MainWIndow::chooseLogFont() {
  // ui->logs->setCurrentFont(
  // QFontDialog::getFont(nullptr, ui->logs->currentFont()));
  ui->logs->setFont(QFontDialog::getFont(nullptr, ui->logs->font()));
}

void MainWIndow::showCurrentImage(int image) {
  if (_images.count(image) == 0) {
    auto q_image = _sfm->getImageFromDB(image);
    _images.emplace(image, QPixmap::fromImage(q_image));
  }
  int w = ui->current_sfm_image->width(), h = ui->current_sfm_image->height();
  ui->current_sfm_image->setPixmap(
      _images.at(image).scaled(w, h, Qt::KeepAspectRatio));
}
