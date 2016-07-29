#include "slamviewer.h"
#include <QtWidgets>

int main(int argc, char *argv[]) {
    QApplication a (argc, argv);
    SLAMViewer w;
    w.show ();
    return a.exec ();
}
