#pragma once
#include "DataFlowObject.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "MVSConfig.h"

/**
 * @brief this is the base class of all types of cameras, the customized camera type should inherit this class,
 * and implement the virtual methods in this class and OpenCV VideoCapture class.
 * 
 */
class CameraObject : public DataFlowObject, public cv::VideoCapture
{
public:
/**
 * @brief Construct a new Camera Object object
 * 
 * @param CameraName the unique name of Camera
 */
	CameraObject();

	/**
	 * @brief Construct a new Camera Object object
	 * 
	 * @param CameraName camera name
	 */
	CameraObject(std::string CameraName);

/**
 * @brief Destroy the Camera Object object
 * 
 */
    virtual ~CameraObject();

/**
 * @brief Get the Camera Name
 * 
 * @return std::string Camera name
 */
	std::string getCameraName();

/**
 * @brief Transforms an image to compensate for lens distortion. 
 * this is a pure virtual function, the real undistort algorithm should be implemented
 *
 * @param src Input (distorted) image.
 * @param dst Output (corrected) image that has the same size and type as src.
 * @return true successfully undistort
 * @return false undistort failed
 */
	virtual bool undistort(const cv::Mat& src, cv::Mat& dst) = 0;
	
	/**
	 * @brief open the camera with the preloaded parameter
	 * 
	 * @return true 
	 * @return false 
	 */
	virtual bool open();

	bool save(JsonNode& fs) override;

	bool load(JsonNode& fs) override;

	virtual std::string type_name() override;

public:
/**
 * @brief shared pointer
 * 
 */
	using Ptr = std::shared_ptr<CameraObject>;

protected:
	std::string CameraName;
	cv::Size2i ImageSize;
private:
	int CameraIndex; // this is for opencv camera open method only
};


/**
 * @brief this is the base class of all pinhole cameras, which is the common camera type.
 * this class implement the common pinhole camera module, which include classic 3x3 camera intrinsic matrix, and distortion module
 * the detailed information of this camera module can be found in OpenCV document "Camera Calibration and 3D Reconstruction" part
 * 
 */
class PinholeCamera : public CameraObject
{
public:
/**
 * @brief Construct a new Pinhole Camera object
 * 
 * @param CameraName the unique name of Camera
 * @param CameraMatrix the 3x3 camera intrinsic matrix
 * @param DistCoeff the camera distortion parameters
 */
	PinholeCamera(const cv::Mat1d& CameraMatrix=cv::Mat1d(), const cv::Mat1d& DistCoeff=cv::Mat1d());

	/**
	 * @brief Destroy the Pinhole Camera object
	 * 
	 */
	virtual ~PinholeCamera();

	virtual bool undistort(const cv::Mat& src, cv::Mat& dst) override;

public:
/**
 * @brief the shared pointer of PinholeCamera class
 * 
 */
	using Ptr = std::shared_ptr<PinholeCamera>;

	/**
	 * @brief Creates PinholeCamera object.
	 * 
 	 * @param CameraName the unique name of Camera
	 * @param CameraMatrix the 3x3 camera intrinsic matrix
 	 * @param DistCoeff the camera distortion parameters
	 * @return Ptr the shared pointer of PinholeCamera class
	 */
	static Ptr Create(const cv::Mat1d& CameraMatrix=cv::Mat1d(), const cv::Mat1d& DistCoeff=cv::Mat1d());

	/**
	 * @brief Get the Camera Parameters
	 * 
	 * @return std::tuple<cv::Mat1d, cv::Mat1d> Camera intrinsic matrix and distortion parameters
	 */
	std::tuple<cv::Mat1d, cv::Mat1d> getCameraParameters();

	/**
	 * @brief set the new camera intrinsic matrix
	 * 
	 * @param CameraMatrix new camera intrinsic matrix
	 * @return true update
	 * @return false update failed
	 */
	bool updateCameraMatrix(const cv::Mat1d& CameraMatrix);

	/**
	 * @brief 
	 * 
	 * @param DistCoeff set the new camera distortion parameters
	 * @return true 
	 * @return false 
	 */
	bool updateDistCoeff(const cv::Mat1d& DistCoeff);

	virtual std::string type_name() override;

	virtual bool save(JsonNode& fs) override;
	virtual bool load(JsonNode& fs) override;

protected:
/**
 * @brief Camera Intrinsic Matrix
 * 
 */
	cv::Mat1d CameraMatrix;

	/**
	 * @brief Camera Distortion Parameters
	 * 
	 */
	cv::Mat1d DistCoeff;

private:
	cv::Mat UndistortMap1, UndistortMap2;
	bool initParameters();
};


