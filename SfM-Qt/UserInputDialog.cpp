#include "UserInputDialog.h"

#include "ui_UserInputDialog.h"

UserInputDialog::UserInputDialog(QWidget* parent)
    : QDialog(parent), ui(new Ui::UserInputDialog) {
  ui->setupUi(this);
}

UserInputDialog::~UserInputDialog() { delete ui; }
