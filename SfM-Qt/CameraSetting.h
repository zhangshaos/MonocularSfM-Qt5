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

 private slots:

 private:
  Ui::CameraSetting *ui;
};
