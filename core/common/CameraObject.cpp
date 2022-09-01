#include "CameraObject.h"

CameraObject::CameraObject()
{
}

CameraObject::~CameraObject()
{
}

std::string CameraObject::getCameraName()
{
	return this->CameraName;
}

bool CameraObject::save(JsonNode& fs)
{
	//TODO: add read images
	return false;
}

bool CameraObject::load(JsonNode& fn)
{
	//TODO: add save images
	return false;
}

PinholeCamera::Ptr PinholeCamera::Create(const cv::Mat1d& CameraMatrix, const cv::Mat1d& DistCoeff)
{
	return std::make_shared<PinholeCamera>(CameraMatrix, DistCoeff);
}

PinholeCamera::PinholeCamera(const cv::Mat1d& CameraMatrix, const cv::Mat1d& DistCoeff)
	:CameraMatrix(CameraMatrix),
	DistCoeff(DistCoeff)
{
}

PinholeCamera::~PinholeCamera()
{
}

bool PinholeCamera::undistort(const cv::Mat& src, cv::Mat& dst)
{
	return false;
}

std::tuple<cv::Mat1d, cv::Mat1d> PinholeCamera::getCameraParameters()
{
	return { CameraMatrix,DistCoeff }; // this syntex {} is introduced in cpp 17
}

bool PinholeCamera::updateCameraMatrix(const cv::Mat1d& CameraMatrix)
{
	if (CameraMatrix.size() != cv::Size(3, 3))
		return false;
	else
		this->CameraMatrix = CameraMatrix;
	return true;
}

bool PinholeCamera::updateDistCoeff(const cv::Mat1d& DistCoeff)
{
	if (DistCoeff.empty())
		return false;
	else
		this->DistCoeff = DistCoeff;
	return true;
}

