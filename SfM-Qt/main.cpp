#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <QVTKOpenGLWidget.h>

#include <QSurfaceFormat>
#include <QtWidgets/QApplication>

#include "MainWIndow.h"



int main(int argc, char *argv[]) {
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());
  QApplication a(argc, argv);
  MainWIndow w;
  w.show();
  return a.exec();
}
