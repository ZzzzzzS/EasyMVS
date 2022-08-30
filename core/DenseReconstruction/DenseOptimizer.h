#pragma once
#include "MVSConfig.h"
#include "WorkFlowObject.h"
#include "FrameObject.h"
#include "DenseMapObject.h"

#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <QObject>

class DenseOptimizer : public WorkFlowObject
{
public:
	Q_OBJECT
	using Ptr = std::shared_ptr<DenseOptimizer>;
	DenseOptimizer::Ptr Create();
public:
	DenseOptimizer();
	~DenseOptimizer();

	

private:

};

