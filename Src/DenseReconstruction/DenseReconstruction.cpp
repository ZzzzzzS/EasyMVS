#include "DenseReconstruction.h"

DenseReconstruction::DenseReconstruction(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	connect(this->ui.pushButton, SIGNAL(clicked()),this,SLOT(pushButton_test()));

	std::string path = ".//params//param.xml";
	params = new CoordinateConvertParams(path);
}

void DenseReconstruction::pushButton_test()
{
	Coordinate_P p1 = Coordinate_P(params);
	Coordinate_Q q1 = Coordinate_Q(params);
	Coordinate_X  x1 = Coordinate_X(params);
	BasicPoint point;
	point.p1 = 600;
	point.p2 = 500;
	point.p3 = 470;
	p1.Points.push_back(point);
	q1.convertQFrom_P(p1);
	x1.convertXFrom_P(p1);

	qDebug() << "p:" << p1.Points[0].p1 << "," << p1.Points[0].p2 << "," << p1.Points[0].p3 ;
	qDebug() << "q:" << q1.Points[0].q1 << "," << q1.Points[0].q2 << "," << q1.Points[0].q3 << "," << q1.Points[0].q4 ;
	qDebug() << "x:" << x1.Points[0].x1 << "," << x1.Points[0].x2 << "," << x1.Points[0].x3 ;

}