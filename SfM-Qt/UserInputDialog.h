#pragma once

#include <QDialog>

namespace Ui {
class UserInputDialog;
}

class UserInputDialog : public QDialog {
  Q_OBJECT
 public:
  UserInputDialog(QWidget *parent);
  ~UserInputDialog();

 private slots:

 protected:
  /**
   * @brief derived class should  add widgets by itself.\n
   * there are only a ListWidget-Container and Ok-Cancel button by default.
  */
  Ui::UserInputDialog *ui;
};
