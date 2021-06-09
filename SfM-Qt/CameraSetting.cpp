#include "CameraSetting.h"

#include "ui_CameraSetting.h"

CameraSetting::CameraSetting(QWidget* parent)
    : QDialog(parent), ui(new Ui::CameraSetting) {
  ui->setupUi(this);
}

CameraSetting::~CameraSetting() { delete ui; }

std::tuple<double, double, double, double> CameraSetting::getConf() const {
  return std::make_tuple(ui->fx_setting->value(), ui->fy_setting->value(),
                         ui->cx_setting->value(), ui->cy_setting->value());
}