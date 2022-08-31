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
	CameraObject(const std::string CameraName);

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
	 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
	 * 
	 * @param fs 
	 * @return true 
	 * @return false 
	 */
	bool save(JsonNode& fs) override;

	/**
	 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
	 * 
	 * @param fs 
	 * @return true 
	 * @return false 
	 */
	bool load(JsonNode& fs) override;

public:
/**
 * @brief shared pointer
 * 
 */
	using Ptr = std::shared_ptr<CameraObject>;

protected:
	const std::string CameraName;
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
	PinholeCamera(const std::string CameraName, const cv::Mat1d& CameraMatrix, const cv::Mat1d& DistCoeff);

	/**
	 * @brief Destroy the Pinhole Camera object
	 * 
	 */
	virtual ~PinholeCamera();

	/**
	 * @brief Transforms an image to compensate for lens distortion. 
	 * this function use the same distortion module as OpenCV, 
	 * the detailed undistort algorithm can be found in OpenCV document 
	 * "Camera Calibration and 3D Reconstruction" part
	 * 
	 * @param src Input (distorted) image.
	 * @param dst Output (corrected) image that has the same size and type as src.
	 * @return true successfully undistort
	 * @return false undistort failed
	 */
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
	static Ptr Create(const std::string CameraName, const cv::Mat1d& CameraMatrix, const cv::Mat1d& DistCoeff);

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
};


