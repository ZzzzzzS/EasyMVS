#include "DualFrameReconstructor.h"

DualFrameReconstructor::Ptr DualFrameReconstructor::Create(GlobalMapObject::Ptr GlobalMap)
{
	return std::make_shared<DualFrameReconstructor>(GlobalMap);
}

DualFrameReconstructor::DualFrameReconstructor(GlobalMapObject::Ptr GlobalMap)
    :DenseReconstructor(GlobalMap)
{
	this->MatcherPtr = cv::StereoSGBM::create();
}

DualFrameReconstructor::~DualFrameReconstructor()
{
}

bool DualFrameReconstructor::clear()
{
    return false;
}

bool DualFrameReconstructor::Compute(FrameObject::Ptr frame)
{
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
		Pinhole1->XYZMat = cv::Mat(XYZMat_);
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
