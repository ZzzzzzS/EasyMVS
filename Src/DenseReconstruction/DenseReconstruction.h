#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_DenseReconstruction.h"

#include "Coordinate_PointTypes.h"

class DenseReconstruction : public QMainWindow
{
    Q_OBJECT

public:
    DenseReconstruction(QWidget *parent = Q_NULLPTR);

private:
    Ui::DenseReconstructionClass ui;

public	slots:
	void pushButton_test();
};
