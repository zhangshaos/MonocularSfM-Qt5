#pragma once

#include <QDialog>
#include <QPixmap>
#include <unordered_map>

namespace Ui {
class InitialPairSetting;
}

class InitialPairSetting : public QDialog {
  Q_OBJECT
 public:
  InitialPairSetting(QWidget *parent);
  ~InitialPairSetting();

  void setInitImagePairs(const std::vector<std::pair<int, int>> &);
  void addImage(int id, const QImage &);

 private slots:
  void showImagePair();
  void accept() override;

 signals:
  void image(int);
  void imagePair(int, int);

 private:
  Ui::InitialPairSetting *ui;
  void initSignalSlot();

  std::unordered_map<int, QPixmap> _images;
};
