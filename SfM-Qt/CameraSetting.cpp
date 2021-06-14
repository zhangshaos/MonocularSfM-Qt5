#include "CameraSetting.h"

#include "ui_CameraSetting.h"

CameraSetting::CameraSetting(QWidget* parent)
    : QDialog(parent), ui(new Ui::CameraSetting) {
  ui->setupUi(this);
}

CameraSetting::~CameraSetting() { delete ui; }

std::vector<double> CameraSetting::getConf() const {
  return std::vector<double>({ui->fx_setting->value(), ui->fy_setting->value(),
                              ui->cx_setting->value(), ui->cy_setting->value(),
                              ui->k1_set->value(), ui->k2_set->value(),
                              ui->p1_set->value(), ui->p2_set->value()});
}