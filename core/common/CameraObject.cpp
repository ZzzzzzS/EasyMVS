#include "CameraObject.h"
#include "JsonSaver.hpp"

CameraObject::CameraObject()
	:CameraIndex(-1)
{
}

CameraObject::~CameraObject()
{
}

CameraObject::CameraObject(std::string CameraName)
	:CameraName(CameraName),
	CameraIndex(-1)
{
}

std::string CameraObject::getCameraName()
{
	return this->CameraName;
}

bool CameraObject::open() 
{
	if(!this->isOpened())
	{
		if(this->CameraIndex>=0)
		{
			return VideoCapture::open(this->CameraIndex);
		}
		else
		{
			return VideoCapture::open(this->CameraName);
		}
	}
	return this->isOpened();
}

bool CameraObject::save(JsonNode& fs)
{
	try
	{
		fs["type-id"]=this->type_name();
		fs["camera-name"]=this->CameraName;
		fs["camera-index"]=this->CameraIndex;
		fs["image-size"]=this->ImageSize;

		return true;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}

	return false;
}

bool CameraObject::load(JsonNode& fs)
{
	try
	{
		this->CameraName=fs.at("camera-name").get<std::string>();
		this->CameraIndex=fs.at("camera-index").get<int>();

		this->ImageSize=fs.at("image-size").get<cv::Size2i>();

		return true;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	return false;
}

std::string CameraObject::type_name() 
{
	return std::string("cv-camera");
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
	if(src.empty())
	{
		return false;
	}

	if(this->UndistortMap1.empty()||this->UndistortMap2.empty())
	{
		return false;
	}

	cv::remap(src, dst, this->UndistortMap1, this->UndistortMap2, cv::INTER_LINEAR);

	return true;
}

std::tuple<cv::Mat1d, cv::Mat1d> PinholeCamera::getCameraParameters()
{
	return { CameraMatrix,DistCoeff }; // this syntex {} is introduced in cpp 17
}

bool PinholeCamera::updateCameraMatrix(const cv::Mat1d& CameraMatrix)
{
	if (CameraMatrix.size() != cv::Size(3, 3))
	{
		std::cout << this->type_name() + ": Intrisic matrix is illegal!" << std::endl;
		return false;
	}
	else
		this->CameraMatrix = CameraMatrix;
	return true;
}

bool PinholeCamera::updateDistCoeff(const cv::Mat1d& DistCoeff)
{
	if (DistCoeff.empty())
	{
		std::cout << this->type_name() + ": Distortion matrix is empty!" << std::endl;
		return false;
	}
	else
		this->DistCoeff = DistCoeff;
	return true;
}

std::string PinholeCamera::type_name() 
{
	return std::string("cv-pinhole-camera");
}

bool PinholeCamera::save(JsonNode& fs) 
{
	// save the base class
	if(!CameraObject::save(fs))
		return false;
	try
	{
		fs["type-id"]=this->type_name();
		fs["camera-matrix"]=this->CameraMatrix;
		fs["dist-coeff"]=this->DistCoeff;
		return true;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	return false;
}

bool PinholeCamera::load(JsonNode& fs) 
{
	if(!CameraObject::load(fs))
		return false;
	try
	{
		this->CameraMatrix=fs.at("camera-matrix").get<cv::Mat1d>();
		this->DistCoeff=fs.at("dist-coeff").get<cv::Mat1d>();

		return this->initParameters();
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	return false;
}

bool PinholeCamera::initParameters() 
{
	if (CameraMatrix.size() != cv::Size(3, 3))
		return false;
	if (DistCoeff.empty())
		return false;
	
	// init the parameters
	cv::initUndistortRectifyMap(CameraMatrix, DistCoeff, cv::Mat(),
	 CameraMatrix, ImageSize, CV_32FC1, this->UndistortMap1, this->UndistortMap2);

	return true;
}

