#include "simpleviewer.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  SimpleViewer w;
  w.show ();

  return a.exec ();
}
