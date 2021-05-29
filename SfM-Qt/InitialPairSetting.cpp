#include "InitialPairSetting.h"

#include <QFormLayout>
#include <QLabel>
#include <QSpinBox>
#include <limits>

#include "ui_UserInputDialog.h"

InitialPairSetting::InitialPairSetting(QWidget* parent)
    : UserInputDialog(parent) {
  setupUi();
}

InitialPairSetting::~InitialPairSetting() {}

void InitialPairSetting::setupUi() {
  /* ________________________________
   * |    图片1    |__________|     |
   * |    图片2    |__________|     |
   * |_______________ |OK|_|CANCEL|_|
   */
  auto label1 = new QLabel("第一张图片"), label2 = new QLabel("第二张图片");
  auto item1 = new QSpinBox, item2 = new QSpinBox;
  item1->setObjectName("initial_image1");
  item1->setMinimum(0);
  item1->setMaximum(std::numeric_limits<int>::max());
  item1->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
  item2->setProperty("objectName", "initial_image2");
  item2->setProperty("minimum", 0);
  item2->setProperty("maximum", std::numeric_limits<int>::max());
  item2->setProperty("sizePolicy",
                     QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred));
  auto layout = new QFormLayout;
  layout->addRow(label1, item1);
  layout->addRow(label2, item2);
  ui->frame->setLayout(layout);
}
