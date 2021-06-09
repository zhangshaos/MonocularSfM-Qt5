#pragma once

#include <QDialog>
#include <any>

namespace Ui {
class RuntimeSetting;
}

class RuntimeSetting : public QDialog {
  Q_OBJECT

 public:
  RuntimeSetting(QWidget *parent = nullptr);
  ~RuntimeSetting();

  std::any getConf() const;

 signals:
  void conf(const std::any &);

 private slots:

 private:
  Ui::RuntimeSetting *ui;
  void setupUi();
};
