#pragma once

#include <QDialog>

namespace Ui {
class CameraSetting;
}

class CameraSetting : public QDialog {
  Q_OBJECT
 public:
  CameraSetting(QWidget *parent);
  ~CameraSetting();

  std::tuple<double, double, double, double> getConf() const;

 signals:
  void conf(double fx, double fy, double cx, double cy);

 public slots:

 private:
  Ui::CameraSetting *ui;
};
