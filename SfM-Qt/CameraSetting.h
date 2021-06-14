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

  std::vector<double> getConf() const;

 signals:
  void conf(const std::vector<double> &conf);

 public slots:

 private:
  Ui::CameraSetting *ui;
};
