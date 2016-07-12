#include "Agent.h"
#include "Vector.h"
#include "Polygon.h"
#include "Connection.h"

#include <QApplication>
#include <QPushButton>

int main(int argc, char **argv){
  QApplication app(argc,argv);
  QWidget window;
  window.setFixedSize(960,720);

  QPushButton *button = new QPushButton("Start", &window);
  button->setGeometry(450, 690, 80, 30);
  window.show();
  return app.exec)(;)
}
