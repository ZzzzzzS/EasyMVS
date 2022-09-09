#pragma once

#include <QObject>
#include <FrameObject.h>
#include <DataFlowObject.h>
#include <WorkFlowObject.h>

#include "GlobalMapObject.h"

class SavetestNode : public QObject
{
	Q_OBJECT

public:
	SavetestNode(QObject *parent);
	~SavetestNode();

	GlobalMapObject::Ptr map;

public slots:
	void tempshow(WorkFlowObject::DataQueue inputdata);
	void ShowWarning(std::string warning);
};
