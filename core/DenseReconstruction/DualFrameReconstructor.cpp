#include "DualFrameReconstructor.h"

DualFrameReconstructor::Ptr DualFrameReconstructor::Create(GlobalMapObject::Ptr GlobalMap)
{
	return std::make_shared<DualFrameReconstructor>(GlobalMap);
}

DualFrameReconstructor::DualFrameReconstructor(GlobalMapObject::Ptr GlobalMap)
    :DenseReconstructor(GlobalMap)
{
}

DualFrameReconstructor::~DualFrameReconstructor()
{
}

bool DualFrameReconstructor::clear()
{
	this->MatcherPtr->clear();
	this->m_isInit = false;
	return true;
}

bool DualFrameReconstructor::Compute(FrameObject::Ptr frame)
{
	if (this->m_isInit == false)
	{
		std::cout << this->type_name() << ": this workflow is not initialized!" << std::endl;
		return false;
	}
	try
	{
		auto Pinhole1 = std::dynamic_pointer_cast<PinholeFrameObject>(frame);
		auto Pinhole2 = std::dynamic_pointer_cast<PinholeFrameObject>(frame->getBestFrame()->getRelatedFrame());
		auto T12 = Pinhole2->getGlobalPose().inverse() * Pinhole1->getGlobalPose();
		cv::Mat1d T12_Mat;
		DataFlowObject::Sophus2cvMat(T12, T12_Mat);
		auto [R, t] = DataFlowObject::T2Rt(T12_Mat);
		auto ImgSize = Pinhole1->RGBMat.size();
		cv::Mat1d R1, R2, P1, P2, Q;
		cv::Rect ROI1, ROI2;
		
		cv::stereoRectify(Pinhole1->CameraMatrix, Pinhole1->DistCoeff,
			Pinhole2->CameraMatrix, Pinhole2->DistCoeff, ImgSize,
			R, t, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, cv::Size(),
			&ROI1, &ROI2);

		cv::Mat map1x, map1y, map2x, map2y;
		cv::initUndistortRectifyMap(Pinhole1->CameraMatrix, cv::noArray(), R1, P1, ImgSize, CV_32FC1, map1x, map1y);
		cv::initUndistortRectifyMap(Pinhole2->CameraMatrix, cv::noArray(), R2, P2, ImgSize, CV_32FC1, map2x, map2y);
		
		cv::Mat Pinhole1Rectified, Pinhole2Rectified;
		cv::remap(Pinhole1->RGBMat, Pinhole1Rectified, map1x, map1y, cv::INTER_LINEAR);
		cv::remap(Pinhole2->RGBMat, Pinhole2Rectified, map2x, map2y, cv::INTER_LINEAR);
		cv::Mat tmp1, tmp2;
		
		cv::Mat disparity;
		this->MatcherPtr->compute(Pinhole1Rectified, Pinhole2Rectified, disparity);
		cv::Mat XYZMat;
		cv::reprojectImageTo3D(disparity, XYZMat, Q);
		
		cv::Mat_<cv::Point3f> XYZMat_(XYZMat);
		cv::Mat1f R1_inv = R1.inv();
		for (auto& item : XYZMat_)
		{
			cv::Mat1f tmp = R1_inv * cv::Mat1f(item);
			item = cv::Point3f(tmp(0), tmp(1), tmp(2));
		}

		Pinhole1->XYZMat = XYZMat;
		//Pinhole1->XYZMat = this->DepthMapFilter(XYZMat);

		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr <<this->type_name()<<": " << e.what() << std::endl;
	}
	return false;
}

bool DualFrameReconstructor::Compute(FrameObject::Ptr frame1, FrameObject Frame2)
{
    return false;
}

bool DualFrameReconstructor::load(JsonNode& fs)
{
	try
	{
		this->minDisparity		= fs.at("min-disparity");
		this->numDisparities	= fs.at("num-disparities");
		this->blockSize			= fs.at("block-size");
		this->p1				= fs.at("p1");
		this->p2				= fs.at("p2");
		this->disp12MaxDiff		= fs.at("disp12-max-diff");
		this->preFilterCap		= fs.at("pre-filter-cap");
		this->uniquenessRatio	= fs.at("uniqueness-ratio");
		this->speckleWindowSize	= fs.at("speckle-window-size");
		this->speckleRange		= fs.at("speckle-range");

		this->MaxDepth			= fs.at("max-depth");
		this->MinDepth			= fs.at("min-depth");

		this->MatcherPtr = cv::StereoSGBM::create(
			minDisparity, numDisparities,
			blockSize, p1, p2, disp12MaxDiff,
			preFilterCap, uniquenessRatio, speckleWindowSize,
			speckleRange);
		this->m_isInit= (this->MatcherPtr == nullptr) ? false : true;
		return (this->MatcherPtr == nullptr) ? false : true;
	}
	catch (const std::exception& e)
	{
		std::cerr << this->type_name() << ": " << e.what() << std::endl;
	}
	return false;
}

bool DualFrameReconstructor::save(JsonNode& fs)
{
	try
	{
		auto ptr = std::dynamic_pointer_cast<cv::StereoMatcher>(this->MatcherPtr);
		if (ptr == nullptr)
		{
			std::cout << this->type_name() << ": Workflow is not initialized, default parameters will be saved" << std::endl;
		}

		fs["algorithm-name"]	 = std::string("opencv-sgbm");
		fs["min-disparity"]		 = this->minDisparity;
		fs["num-disparities"]	 = this->numDisparities;
		fs["block-size"]		 = this->blockSize;
		fs["p1"]				 = this->p1;
		fs["p2"]				 = this->p2;
		fs["disp12-max-diff"]	 = this->disp12MaxDiff;
		fs["pre-filter-cap"]	 = this->preFilterCap;
		fs["uniqueness-ratio"]	 = this->uniquenessRatio;
		fs["speckle-window-size"]= this->speckleWindowSize;
		fs["speckle-range"]		 = this->speckleRange;

		fs["max-depth"]			 = this->MaxDepth;
		fs["min-depth"]			 = this->MinDepth;

		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << this->type_name() << ": " << e.what() << std::endl;
		fs = JsonNode::value_t::null;
	}
	return false;
}

cv::Mat DualFrameReconstructor::DepthMapFilter(const cv::Mat& depth_map)
{
	cv::Mat_<cv::Point3f> OutMap = cv::Mat_<cv::Point3f>::zeros(depth_map.size());
	cv::Mat_<cv::Point3f> InMap(depth_map);

	for (size_t i = 0; i < InMap.rows; i++)
	{
		for (size_t j = 0; j < InMap.cols; j++)
		{
			auto point = InMap(i, j);
			if ((point.z > this->MinDepth) && (point.z < this->MaxDepth))
			{
				OutMap(i, j) = point;
			}
				
		}
	}

	return OutMap;
}
