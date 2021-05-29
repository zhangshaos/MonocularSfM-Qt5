#include "MainWIndow.h"

#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>

#include "ui_MainWIndow.h"
#include "ui_OptimizeSetting.h"

MainWIndow::MainWIndow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWIndowClass),
      _optimize_set(new OptimizeSetting(this)),
      _camara_set(new CameraSetting(this)),
      _runtime_set(new RuntimeSetting(this)),
      _init_pair_set(new InitialPairSetting(this)),
      _db_shower(new DB_Shower(nullptr /*independent window*/)) {
  ui->setupUi(this);
  initSignalSlot();
}

void MainWIndow::initSignalSlot() const {
  connect(ui->action_choose_pair, SIGNAL(triggered()), this,
          SLOT(buildInitPair()));
  connect(ui->action_camera_conf, SIGNAL(triggered()), this,
          SLOT(confCamera()));
  connect(ui->action_run_conf, SIGNAL(triggered()), this, SLOT(confRuntime()));
}

MainWIndow::~MainWIndow() { delete ui; }

void MainWIndow::dataOpenFold() {
  qDebug("action open fold is clicked\n");
  QFileDialog dialog(this, "打开图片文件夹");
  dialog.setFileMode(QFileDialog::Directory);
  if (dialog.exec()) {
    auto d = dialog.directory();
    qDebug("selected path is %s,\nabsolute path is %s,\ncanonical path is %s\n",
           qPrintable(d.path()), qPrintable(d.absolutePath()),
           qPrintable(d.canonicalPath()));
    // TODO: load images!
  } else
    qDebug("choose nothing...\n");
}

void MainWIndow::dataOpenDB() {
  qDebug("action open DB is clicked\n");
  QFileDialog dialog(this,
                     "打开图片数据库和图片关系数据（按 CTRL 键进行多选）");
  dialog.setFileMode(QFileDialog::ExistingFiles);
  if (dialog.exec()) {
    auto names = dialog.selectedFiles();
    if (names.size() != 2) {
      QMessageBox::warning(
          this, "警告",
          "必须选择两个文件：一个为图片数据库，一个为图片关系数据");
      return;
    }
    // TODO: load DB
    qDebug("file names are:\n%s\n%s\n", qPrintable(names.at(0)),
           qPrintable(names.at(1)));
  } else
    qDebug("choose nothing...\n");
}

void MainWIndow::buildContinue() {
  qDebug("action continue build is clicked\n");
  // ui->logs->insertPlainText("insertPlainText");
  // TODO: continue build
  ui->logs->append("append");
}

void MainWIndow::buildOptimize() {
  qDebug("action set optimize is clicked\n");
  if (_optimize_set->exec()) {
    auto m = _optimize_set->mode();
    // TODO: BA setting
  }
}

void MainWIndow::buildInitPair() {
  qDebug("action %s is clicked\n",
         qPrintable(ui->action_choose_pair->objectName()));
  //弹出图片菜单
  //if (_db_shower->isHidden()) {
  //  _db_shower->show();
  //}
  //选择图片对
  if (_init_pair_set->exec()) {
    // TODO: initial pair setting
  }
}

void MainWIndow::confCamera() {
  qDebug("action %s is clicked\n",
         qPrintable(ui->action_camera_conf->objectName()));
  //弹出相机参数菜单
  if (_camara_set->exec()) {
    // TODO: camera setting
  }
}

void MainWIndow::confRuntime() {
  qDebug("action %s is clicked\n",
         qPrintable(ui->action_run_conf->objectName()));
  //弹出设置菜单
  if (_runtime_set->exec()) {
    // TODO: runtime setting
  }
}
