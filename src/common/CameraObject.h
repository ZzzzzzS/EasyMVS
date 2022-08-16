#pragma once
#include "DataFlowObject.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "MVSConfig.h"

class CameraObject : public DataFlowObject, public cv::VideoCapture
{
public:
	CameraObject(const std::string CameraName);
    virtual ~CameraObject();
	std::string getCameraName();
	virtual bool undisort(const cv::Mat& src, cv::Mat& dst) = 0;
	
	bool save(Json& fs) override;
	bool load(Json& fs) override;

public:
	using Ptr = std::shared_ptr<CameraObject>;
	//static Ptr Create(std::string CameraName);

protected:
	const std::string CameraName;
};

class PinholeCamera : public CameraObject
{
public:
	PinholeCamera(const std::string CameraName, const cv::Mat1d& CameraMatrix, const cv::Mat1d& DistCoeff);
	virtual ~PinholeCamera();
	virtual bool undisort(const cv::Mat& src, cv::Mat& dst);
public:
	using Ptr = std::shared_ptr<PinholeCamera>;
	static Ptr Create(const std::string CameraName, const cv::Mat1d& CameraMatrix, const cv::Mat1d& DistCoeff);

	std::tuple<cv::Mat1d, cv::Mat1d> getCameraParameters();
	bool updateCameraMatrix(const cv::Mat1d& CameraMatrix);
	bool updateDistCoeff(const cv::Mat1d& DistCoeff);

protected:
	cv::Mat1d CameraMatrix;
	cv::Mat1d DistCoeff;
};


