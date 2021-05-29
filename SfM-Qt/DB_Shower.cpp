#include "DB_Shower.h"

#include "ui_DB_Shower.h"

DB_Shower::DB_Shower(QWidget* parent) : QWidget(parent), ui(new Ui::DB_Shower) {
  ui->setupUi(this);
}

DB_Shower::~DB_Shower() { delete ui; }
