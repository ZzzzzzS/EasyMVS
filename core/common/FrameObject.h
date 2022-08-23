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
	bool setBestCamera(FrameObject::Ptr Camera, const Sophus::SE3d& Pose);
	
	/**
	 * @brief 
	 * 
	 * @param Frame 
	 * @param Pose 
	 * @return true 
	 * @return false 
	 */
	bool addRelatedFrame(FrameObject::Ptr Frame, const Sophus::SE3d& Pose);


	/**
	 * @brief 
	 * 
	 * @param Frame 
	 * @param Pose 
	 * @return true 
	 * @return false 
	 */
	bool updateRelatedFrame(FrameObject::Ptr Frame, const Sophus::SE3d& Pose);

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

	/**
	 * @brief Get the All Related Frames object
	 * 
	 * @param Frames 
	 * @return true has related frames
	 * @return false do NOT have related frames
	 */
	bool getAllRelatedFrames(std::vector<FrameObject::Ptr>& Frames);


/**
 * @brief add frame observed map point
 * 
 * @param KeyPointID map point related key point ID
 * @param MapPoint Pointer of map point
 * @param LocalCoordinate map point coordinate in the current camera frame
 * @return true add successfully
 * @return false add failed
 */
	bool addMapPoint(int KeyPointID, std::shared_ptr<MapPointObject> MapPoint, const Eigen::Vector4d& LocalCoordinate);

	/**
	 * @brief remove map point from frame
	 * 
	 * @param KeyPointID the map point related keypoint ID 
	 * @return true remove successfully
	 * @return false remove failed
	 */
	bool removeMapPoint(int KeyPointID);

	/**
	 * @brief remove all map points
	 * 
	 * @return true remove successfully
	 * @return false remove failed
	 */
	bool removeAllMapPoints();

	/**
	 * @brief Get the Map Point object
	 * 
	 * @param KeyPointID input key point ID
	 * @param MapPoint output map point pointer
	 * @param LocalCoordinate output map point coordinate in local frame
	 * @return true get successfully
	 * @return false get failed
	 */
	bool getMapPoint(int KeyPointID, std::shared_ptr<MapPointObject>& MapPoint, Eigen::Vector4d& LocalCoordinate);

	/**
	 * @brief update the map point
	 * 
	 * @param KeyPointID input key point ID
	 * @param MapPoint input map point pointer
	 * @param LocalCoordinate input mappoint coordinate
	 * @return true update successfully
	 * @return false update failed, may because input error, and may also because of the map point dose NOT exsit in the frame
	 */
	bool updateMapPoint(int KeyPointID, std::shared_ptr<MapPointObject> MapPoint, const Eigen::Vector4d& LocalCoordinate);

/**
 * @brief load frame from file
 * 
 * @param fs 
 * @return true load successed 
 * @return false load failed
 */
	bool load(JsonNode& fs) override;

	/**
	 * @brief save frame to file
	 * 
	 * @param fs 
	 * @return true save successed
	 * @return false save failed
	 */
	bool save(JsonNode& fs) override;

public:
/**
 * @brief the RGB image in the current frame
 * the current frame may NOT contain RGB frame, 
 * if the frame is created by projectors, the RGB image is empty.
 */
	cv::Mat RGBMat;

	/**
	 * @brief the depth map in the current frame
	 * the current frame may contain depth map when it is created,
	 * if the frame is created by RGBD cameras,
	 * the original depth map will be stored here when frame is created.
	 */
	cv::Mat XYZMat;

	/**
	 * @brief vector of KeyPoints
	 * 
	 */
	std::vector<cv::KeyPoint> KeyPoints;

	/**
	 * @brief KeyPoints descriptors, the storeage details is the same as it in OpenCV
	 * the details can be seen in OpenCV documents.
	 */
	cv::Mat KeyPointsDescriptors;

	/**
	 * @brief the Bag of Words descriptor of the current frame.
	 */
	DBoW2::BowVector BoWDescriptors;


/**
 * @brief the global pose of current frame
 * 
 */
	Sophus::SE3d GlobalPose;


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

/**
 * @brief Camera intrinsic matrix
 * 
 */
	cv::Mat1d CameraMatrix;

	/**
	 * @brief camera distortion parameters
	 * 
	 */
	cv::Mat1d DistCoeff;
private:

};

