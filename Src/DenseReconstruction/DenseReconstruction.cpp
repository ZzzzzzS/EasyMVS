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
	//Coordinate_P p1 = Coordinate_P(params);
	//Coordinate_Q q1 = Coordinate_Q(params);
	//Coordinate_X  x1 = Coordinate_X(params);
	//Coordinate_X  x2= Coordinate_X(params);

	//BasicPoint point;
	//point.p1 = 600;
	//point.p2 = 500;
	//point.p3 = 300;
	//p1.Points.push_back(point);
	//q1.convertQFrom_P(p1);
	//x1.convertXFrom_P(p1);
	//x2.convertXFrom_Q(q1);

	//qDebug() << "p:" << p1.Points[0].p1 << "," << p1.Points[0].p2 << "," << p1.Points[0].p3 ;
	//qDebug() << "p->q:" << q1.Points[0].q1 << "," << q1.Points[0].q2 << "," << q1.Points[0].q3 << "," << q1.Points[0].q4 ;
	//qDebug() << "p->x1:" << x1.Points[0].x1 << "," << x1.Points[0].x2 << "," << x1.Points[0].x3 ;
	//qDebug() << "q->x2:" << x2.Points[0].x1 << "," << x2.Points[0].x2 << "," << x2.Points[0].x3;

	//Coordinate_P p2 = Coordinate_P(params);

	//p2.convertPFrom_Q(q1);
	////p2.convertPFrom_X(x2);
	//q1.convertQFrom_X(x2);

	//qDebug() << "q->p:" << p2.Points[0].p1 << "," << p2.Points[0].p2 << "," << p2.Points[0].p3;
	//qDebug() << "x2->p:" << p2.Points[0].p1 << "," << p2.Points[0].p2 << "," << p2.Points[0].p3;
	//qDebug() << "x2->q:" << q1.Points[0].q1 << "," << q1.Points[0].q2 << "," << q1.Points[0].q3 << "," << q1.Points[0].q4;

	//p2.convertPFrom_X(x1);
	//q1.convertQFrom_X(x1);

	//qDebug() << "x1->p:" << p2.Points[0].p1 << "," << p2.Points[0].p2 << "," << p2.Points[0].p3;
	//qDebug() << "x1->q:" << q1.Points[0].q1 << "," << q1.Points[0].q2 << "," << q1.Points[0].q3 << "," << q1.Points[0].q4;

	Coordinate_P p1 = Coordinate_P(params);
	Coordinate_Q q1 = Coordinate_Q(params);
	Coordinate_X  x1 = Coordinate_X(params);
	Coordinate_X  x2 = Coordinate_X(params);

	BasicPoint point;
	point.x1 = -50;
	point.x2 = 100;
	point.x3 = 3000;
	x1.Points.push_back(point);

	q1.convertQFrom_X(x1);
	p1.convertPFrom_X(x1);

	x2.convertXFrom_Q(q1);

	qDebug() << "x1:" << x1.Points[0].x1 << "," << x1.Points[0].x2 << "," << x1.Points[0].x3;
	qDebug() << "x1->q:" << q1.Points[0].q1 << "," << q1.Points[0].q2 << "," << q1.Points[0].q3 << "," << q1.Points[0].q4;
	qDebug() << "x1->p:" << p1.Points[0].p1 << "," << p1.Points[0].p2 << "," << p1.Points[0].p3;
	qDebug() << "q->x2:" << x2.Points[0].x1 << "," << x2.Points[0].x2 << "," << x2.Points[0].x3;

	Coordinate_P p2 = Coordinate_P(params);
	q1.convertQFrom_P(p1);
	qDebug() << "p->q:" << q1.Points[0].q1 << "," << q1.Points[0].q2 << "," << q1.Points[0].q3 << "," << q1.Points[0].q4;

	x2.convertXFrom_P(p1);
	qDebug() << "p->x2:" << x2.Points[0].x1 << "," << x2.Points[0].x2 << "," << x2.Points[0].x3;

	p1.convertPFrom_Q(q1);

	p2.convertPFrom_X(x2);
	q1.convertQFrom_P(p1);

	qDebug() << "q->p:" << p1.Points[0].p1 << "," << p1.Points[0].p2 << "," << p1.Points[0].p3;
	qDebug() << "p->q:" << q1.Points[0].q1 << "," << q1.Points[0].q2 << "," << q1.Points[0].q3 << "," << q1.Points[0].q4;

	p1.convertPFrom_Q(q1);
	q1.convertQFrom_P(p1);

	qDebug() << "q->p:" << p1.Points[0].p1 << "," << p1.Points[0].p2 << "," << p1.Points[0].p3;
	qDebug() << "p->q:" << q1.Points[0].q1 << "," << q1.Points[0].q2 << "," << q1.Points[0].q3 << "," << q1.Points[0].q4;

	p2.convertPFrom_X(x1);
	q1.convertQFrom_X(x1);

	qDebug() << "x1->p:" << p2.Points[0].p1 << "," << p2.Points[0].p2 << "," << p2.Points[0].p3;
	qDebug() << "x1->q:" << q1.Points[0].q1 << "," << q1.Points[0].q2 << "," << q1.Points[0].q3 << "," << q1.Points[0].q4;
}