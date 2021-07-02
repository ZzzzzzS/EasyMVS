#pragma once

#include <QtWidgets/QMainWindow>
#include <QDebug>

#include "ui_DenseReconstruction.h"

#include "CoordinateConvertParams.h"
#include "Coordinate_PointTypes.h"

class DenseReconstruction : public QMainWindow
{
    Q_OBJECT

public:
    DenseReconstruction(QWidget *parent = Q_NULLPTR);

private:
    Ui::DenseReconstructionClass ui;

public:
	CoordinateConvertParams* params;//参数体系
public	slots:
	void pushButton_test();
};
