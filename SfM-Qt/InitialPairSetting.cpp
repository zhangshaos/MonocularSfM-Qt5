#include "InitialPairSetting.h"

#include <QMessageBox>

#include "MainWIndow.h"
#include "ui_InitialPairSetting.h"

InitialPairSetting::InitialPairSetting(QWidget* parent)
    : QDialog(parent), ui(new Ui::InitialPairSetting) {
  ui->setupUi(this);
  initSignalSlot();
}

InitialPairSetting::~InitialPairSetting() { delete ui; }

void InitialPairSetting::initSignalSlot() {
  connect(ui->show_image_pair, &QPushButton::clicked, this,
          &InitialPairSetting::showImagePair);
}

void InitialPairSetting::setInitImagePairs(
    const std::vector<std::pair<int, int>>& pairs) {
  using namespace std;
  ui->image_pair->clear();
  for (auto& pr : pairs) {
    auto text = QString("%1 - %2").arg(pr.first).arg(pr.second);
    ui->image_pair->addItem(text, QVariant::fromValue(pr));
  }
}

void InitialPairSetting::addImage(int id, const QImage& img) {
  _images.emplace(id, QPixmap::fromImage(img));
}

void InitialPairSetting::showImagePair() {
  using namespace std;
  auto [id1, id2] = ui->image_pair->currentData().value<pair<int, int>>();
  if (_images.count(id1) == 0) {
    emit image(id1);
    assert(_images.count(id1));
  }
  if (_images.count(id2) == 0) {
    emit image(id2);
    assert(_images.count(id2));
  }
  // TODO: pixelmap's size zooming with zooming of this widget.
  // show two images
  int w = ui->left_pic->width(), h = ui->left_pic->height();
  ui->left_pic->setPixmap(_images.at(id1).scaled(w, h, Qt::KeepAspectRatio));
  w = ui->right_pic->width(), h = ui->right_pic->height();
  ui->right_pic->setPixmap(_images.at(id2).scaled(w, h, Qt::KeepAspectRatio));
}

void InitialPairSetting::accept() {
  using namespace std;
  auto [id1, id2] = ui->image_pair->currentData().value<pair<int, int>>();
  emit imagePair(id1, id2);
  QDialog::accept();
}
