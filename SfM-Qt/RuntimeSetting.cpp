#include "RuntimeSetting.h"

#include "ui_RuntimeSetting.h"

RuntimeSetting::RuntimeSetting(QWidget* parent)
    : QDialog(parent), ui(new Ui::RuntimeSetting) {
  ui->setupUi(this);
}

RuntimeSetting::~RuntimeSetting() { delete ui; }
