#include "CameraSetting.h"

#include "ui_CameraSetting.h"

CameraSetting::CameraSetting(QWidget* parent)
    : QDialog(parent), ui(new Ui::CameraSetting) {
  ui->setupUi(this);
}

CameraSetting::~CameraSetting() { delete ui; }
