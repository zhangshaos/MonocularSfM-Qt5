#pragma once

#include "UserInputDialog.h"

class InitialPairSetting : public UserInputDialog {
  Q_OBJECT
 public:
  InitialPairSetting(QWidget *parent);
  ~InitialPairSetting();

 private slots:

 private:
  void setupUi();
};
