#pragma once
#include "DenseReconstructor.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "MVSConfig.h"

class DualFrameReconstructor : public DenseReconstructor
{
public:
	Q_OBJECT
public:
	using Ptr = std::shared_ptr<DualFrameReconstructor>;
	static DualFrameReconstructor::Ptr Create(GlobalMapObject::Ptr GlobalMap);
public:
	DualFrameReconstructor(GlobalMapObject::Ptr GlobalMap);
	virtual ~DualFrameReconstructor();
	bool clear() override;
	bool Compute(FrameObject::Ptr frame) override;
	bool Compute(FrameObject::Ptr frame1, FrameObject Frame2) override;
	bool load(JsonNode& fs) override;
	bool save(JsonNode& fs) override;
	
private:
	cv::Mat DepthMapFilter(const cv::Mat& depth_map);
	
	int MaxDepth = 1000;
	int MinDepth = 0;

	//SGBM parameters
	int minDisparity = 0;
	int numDisparities = 16;
	int blockSize = 3;
	int p1 = 0;
	int p2 = 0;
	int disp12MaxDiff = 0;
	int preFilterCap = 0;
	int uniquenessRatio = 0;
	int speckleWindowSize = 0;
	int speckleRange = 0;
	
};