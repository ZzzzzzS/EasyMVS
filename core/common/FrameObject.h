#pragma once
#include "MVSConfig.h"
#include "DataFlowObject.h"
#include <opencv2/opencv.hpp>
#include <DBoW2/DBoW2.h>
#include <sophus/se3.hpp>
#include <Eigen/Dense>

class MapPointObject;


/**
 * @brief the is the frame class
 * 
 */
class FrameObject : public DataFlowObject
{
public:
	using Ptr = std::shared_ptr<FrameObject>;

/**
 * @brief Create FrameObject object.
 * 
 * @param ID Unique frame ID
 * @param RGBMat the undistorted RGB image.
 * @param XYZMap the XYZ map or depth map, the RGBD camera may already compute the original depth map when frame is created.
 * @return Ptr 
 */
	static Ptr Create(int ID, const cv::Mat& RGBMat, const cv::Mat& XYZMap = cv::Mat());

	/**
	 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
	 * 
	 * @param ID 
	 * @return Ptr 
	 */
	static Ptr Create(int ID);

public:

/**
 * @brief Construct a new Frame Object object
 * 
 * @param ID unique frame ID
 */
	FrameObject(int ID);

	/**
	 * @brief Destroy the Frame Object object
	 * 
	 */
	virtual ~FrameObject();

public:
/**
 * @brief Get the Frame ID
 * 
 * @return int Frame ID
 */
	int getFrameID();

	/**
	 * @brief Get the Best Camera pair
	 * 
	 * @param Camera the shared pointer of another camera
	 * @param Pose the extrinsic matrix between two cameras
	 * @return true 
	 * @return false 
	 */
	bool getBestCamera(FrameObject::Ptr& Camera, Sophus::SE3d& Pose);

	/**
	 * @brief Set the Best Camera
	 * 
	 * @param Camera the shared pointer of another camera
	 * @param Pose the extrinsic matrix between two cameras
	 * @return true 
	 * @return false 
	 */
	bool setBestCamera(FrameObject::Ptr Camera, const Sophus::SE3d Pose);
	
	/**
	 * @brief 
	 * 
	 * @param Frame 
	 * @param Pose 
	 * @return true 
	 * @return false 
	 */
	bool addRelatedFrame(FrameObject::Ptr Frame, const Sophus::SE3d Pose);

	/**
	 * @brief 
	 * 
	 * @param FrameID 
	 * @return true 
	 * @return false 
	 */
	bool removeRelatedFrame(int FrameID);

	/**
	 * @brief 
	 * 
	 * @return true 
	 * @return false 
	 */
	bool removeAllRelatedFrames();

	/**
	 * @brief Get the Related Frame object
	 * 
	 * @param FrameID 
	 * @return FrameObject::Ptr 
	 */
	FrameObject::Ptr getRelatedFrame(int FrameID);
	bool getAllRelatedFrames(std::vector<FrameObject::Ptr>& Frames);

	bool addMapPoint(int KeyPointID, std::shared_ptr<MapPointObject> MapPoint, const Eigen::Vector4d& LocalCoordinate);
	bool removeMapPoint(int KeyPointID);
	bool removeAllMapPoints();
	bool getMapPoint(int KeyPointID, std::shared_ptr<MapPointObject>& MapPoint, Eigen::Vector4d& LocalCoordinate);
	//TODO: 差更新点

	bool load(JsonNode& fs) override;
	bool save(JsonNode& fs) override;

public:
	cv::Mat RGBMat;
	cv::Mat XYZMat;
	std::vector<cv::KeyPoint> KeyPoints;
	cv::Mat KeyPointsDescriptors;
	DBoW2::BowVector BoWDescriptors;
	Sophus::SE3d GlobalPose;
	Sophus::SE3d LocalPose;


private:
	const int FrameID;
	using RelatedFrameInfo = std::tuple<std::weak_ptr<FrameObject>, Sophus::SE3d>;
	RelatedFrameInfo BestCamera;
	std::map<int, RelatedFrameInfo> RelatedFrame;

	using MapPointInfo = std::tuple<std::weak_ptr<MapPointObject>, Eigen::Vector4d>;
	std::map<int, MapPointInfo> OvservedMapPoints;


};

class PinholeFrameObject : public FrameObject
{
public:
	using Ptr = std::shared_ptr<PinholeFrameObject>;

	/**
	 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
	 * 
	 * @param ID 
	 * @return Ptr 
	 */
	static Ptr Create(int ID);

	/**
	 * @brief Create the PinholeFrameObject object
	 * 
	 * @param ID unique frame ID
	 * @param CameraMatrix Camera intrinsic matrix
	 * @param DistCoeff Camera distortion parameters
	 * @param RGBMat the undistorted RGB image.
	 * @param XYZMap the XYZ map or depth map, the RGBD camera may already compute the original depth map when frame is created.
	 * @return Ptr 
	 */
	static Ptr Create(int ID, 
		const cv::Mat& CameraMatrix,
		const cv::Mat& DistCoeff,
		const cv::Mat& RGBMat = cv::Mat(),
		const cv::Mat& XYZMap = cv::Mat());

public:
/**
 * @brief Construct a new PinholeFrameObject object
 * 
 * @param ID unique frame ID
 */
	PinholeFrameObject(int ID);

	/**
	 * @brief Destroy the Pinhole Frame Object object
	 * 
	 */
	~PinholeFrameObject();

	cv::Mat1d CameraMatrix;
	cv::Mat1d DistCoeff;
private:

};

