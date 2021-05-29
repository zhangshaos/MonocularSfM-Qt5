#pragma once

#include <QWidget>

namespace Ui {
class DB_Shower;
}

class DB_Shower : public QWidget {
  Q_OBJECT
 public:
  DB_Shower(QWidget* parent);
  ~DB_Shower();

 private slots:

 private:
  Ui::DB_Shower *ui;
};
