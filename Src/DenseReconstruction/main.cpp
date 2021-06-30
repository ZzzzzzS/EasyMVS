#include "DenseReconstruction.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DenseReconstruction w;
    w.show();
    return a.exec();
}
