#pragma once

#include <QDialog>

namespace Ui {
class RuntimeSetting;
}

class RuntimeSetting : public QDialog {
  Q_OBJECT

 public:
  RuntimeSetting(QWidget *parent = nullptr);
  ~RuntimeSetting();

 private slots:

 private:
  Ui::RuntimeSetting *ui;
};
