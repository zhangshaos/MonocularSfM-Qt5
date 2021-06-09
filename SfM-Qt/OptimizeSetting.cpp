#include "OptimizeSetting.h"

#include "ui_OptimizeSetting.h"

OptimizeSetting::OptimizeSetting(QWidget* parent)
    : QDialog(parent), ui(new Ui::OptimizeSetting) {
  ui->setupUi(this);
}

OptimizeSetting::~OptimizeSetting() { delete ui; }

void OptimizeSetting::onLocalOptimize(bool ok) {
  qDebug("call localBA: %d\n", ok);
  if (!ok) return;
  _ba_mode = local;
}

OptimizeSetting::OptMode OptimizeSetting::mode() const { return _mode; }

void OptimizeSetting::onGlobalOptimize(bool ok) {
  qDebug("call globalBA: %d\n", ok);
  if (!ok) return;
  _ba_mode = global;
}

void OptimizeSetting::onOptimizeNone(bool ok) {
  qDebug("call optimize none: %d\n", ok);
  if (!ok) return;
  _ba_pose = none;
}

void OptimizeSetting::onOptimizeInitCur(bool ok) {
  qDebug("call optimize init and cur: %d\n", ok);
  if (!ok) return;
  _ba_pose = exclude_init_cur;
}

void OptimizeSetting::onOptimizeAll(bool ok) {
  qDebug("call optimize all: %d\n", ok);
  if (!ok) return;
  _ba_pose = all;
}

void OptimizeSetting::accept() {
  switch (_ba_mode) {
    case local:
      switch (_ba_pose) {
        case exclude_init_cur:
          _mode = OptMode::local_exclude_init_cur;
          break;
        case all:
          _mode = OptMode::local_all;
          break;
        case none:
          _mode = OptMode::local_none;
          break;
        default:
          assert(0);
      }
      break;
    case global:
      switch (_ba_pose) {
        case exclude_init_cur:
          _mode = OptMode::global_exclude_init_cur;
          break;
        case all:
          _mode = OptMode::global_all;
          break;
        case none:
          _mode = OptMode::global_none;
          break;
        default:
          assert(0);
      }
      break;
    default:
      // _mode unchanged
      break;
  }
  qDebug("====== BA optimize mode is %d ======\n", _mode);
  QDialog::accept();
}
