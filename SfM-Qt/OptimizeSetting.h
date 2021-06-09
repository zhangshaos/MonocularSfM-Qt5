#pragma once

#include <QDialog>

namespace Ui {
class OptimizeSetting;
}

class OptimizeSetting : public QDialog {
  Q_OBJECT

 public:
  OptimizeSetting(QWidget *parent = nullptr);
  ~OptimizeSetting();

  enum class OptMode {
    local_none,
    local_exclude_init_cur,
    local_all,
    global_none,
    global_exclude_init_cur,
    global_all
  };
  OptMode mode() const;

 private slots:
  void onLocalOptimize(bool);
  void onGlobalOptimize(bool);
  void onOptimizeNone(bool);
  void onOptimizeInitCur(bool);
  void onOptimizeAll(bool);
  void accept() override;

 private:
  Ui::OptimizeSetting *ui;
  enum { unknown, local, global } _ba_mode = unknown;
  enum { none, exclude_init_cur, all } _ba_pose = none;
  OptMode _mode = OptMode::local_none;
};
