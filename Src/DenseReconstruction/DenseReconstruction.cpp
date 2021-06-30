#include "DenseReconstruction.h"

DenseReconstruction::DenseReconstruction(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	connect(this->ui.pushButton, SIGNAL(clicked()),this,SLOT(pushButton_test()));
}

void DenseReconstruction::pushButton_test()
{
	Coordinate_Q q1;
	Coordinate_X x1;
	q1.convertFrom_X(x1);
}