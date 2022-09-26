#pragma once
#include "MVSConfig.h"
#include "DataFlowObject.h"
#include <opencv2/opencv.hpp>
#include <DBoW2/DBoW2.h>
#include <sophus/se3.hpp>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

class MapPointObject;

class FrameObject : public DataFlowObject
{
public:
	using Ptr = std::shared_ptr<FrameObject>;

/**
 * @brief Create FrameObject object.
 * 
 * @param ID Unique frame ID
 * @param the map ID, which is assigned by GlobalMap
 * @param RGBMat the undistorted RGB image.
 * @param XYZMap the XYZ map or depth map, the RGBD camera may already compute the original depth map when frame is created.
 * @return Ptr 
 */
	static Ptr Create(int ID, int MapID, const cv::Mat& RGBMat, uint32_t Timestamp = 0, const cv::Mat& XYZMat = cv::Mat());

	/**
	 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
	 * 
	 * @param ID 
	 * @return Ptr 
	 */
	static Ptr Create(int ID, int MapID, uint32_t Timestamp = 0);

public:
	/**
	 * @brief Construct a new Frame Object object.
	 */
	FrameObject();

/**
 * @brief Construct a new Frame Object object
 * 
 * @param ID unique frame ID
 */
	FrameObject(int ID, uint32_t Timestamp = 0);

	/**
	 * @brief Destroy the Frame Object object
	 * 
	 */
	virtual ~FrameObject();

public:
	
	/**
	 * @brief the related frame info class, related frame can be defined as the frames that have common
	 * field of views.
	 * which store Interframe information such as pose between two frames, 
	 * prior believe of the pose, and the matched keypoints between frames.
	 */
	class RelatedFrameInfo : public DataFlowObject
	{
	public:
		/**
		 * @brief shared pointer.
		 */
		using Ptr = std::shared_ptr<RelatedFrameInfo>;

		/**
		 * @brief create the shared pointer of relatedframeinfo.
		 * 
		 * @param RelatedFrame shared pointer of related frame.
		 * @return shared pointer, return empty pointer when create failed.
		 */
		static Ptr Create(std::shared_ptr<FrameObject> RelatedFrame);

		/**
		 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts..
		 * 
		 * @param RelatedFrame
		 * @param Pose
		 * @param sigma
		 * @return 
		 */
		static Ptr Create(std::shared_ptr<FrameObject> RelatedFrame, std::shared_ptr<Sophus::SE3d> Pose, double sigma = 0);

		/**
		 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts..
		 * 
		 * @param RelatedFrame
		 * @param KeyPointMatch
		 * @return 
		 */
		static Ptr Create(std::shared_ptr<FrameObject> RelatedFrame,const std::vector<cv::DMatch>& KeyPointMatch);
	public:

		/**
		 * @brief get the pointer of related frame.
		 * 
		 * @return shared pointer of related frame, return empty pointer if the frame is not exist.
		 */
		std::shared_ptr<FrameObject> getRelatedFrame();

		/**
		 * @brief set the shared pointer of related frame.
		 * 
		 * @param frame
		 * @return 
		 */
		bool setRelatedFrame(FrameObject::Ptr frame);

		/**
		 * @brief query wether the frame exist.
		 * usually the frame exist when the frame info exist, but there might be some garbage collection errors, during development.
		 * this method is to make sure that the frame dose exist.
		 * 
		 * @return wether the frame exist.
		 */
		bool isFrameExist();

		bool save(JsonNode& fs) override;

		bool load(JsonNode& fs) override;

		virtual std::string type_name() override;

		/**
		 * @brief Construct a new Related Frame Info object
		 * 
		 */
		RelatedFrameInfo();
		/**
		 * @brief Construct a new Frameinfo object.
		 * instead of call this method, it is better to construct new frame info object with **Create**method.
		 * 
		 * @param RelatedFrame
		 */
		RelatedFrameInfo(std::shared_ptr<FrameObject> RelatedFrame);

		/**
		 * @brief Destroy the object.
		 * 
		 */
		~RelatedFrameInfo();

		/**
		 * @brief shared pointer of pose.
		 * @details the pose is the pose between current frame and related frame, 
		 * many related frame may share the same pose, that is when in pose optimization process,
		 * all the pointer that point to the same pose will be optimized as one coeefficient.
		 */
		std::shared_ptr<Sophus::SE3d> Pose;

		/**
		 * @brief variance of pose.
		 */
		double sigma;

		/**
		 * @brief matched feature points.
		 */
		std::vector<cv::DMatch> KeyPointMatch;

		friend std::ostream& operator<<(std::ostream& os, FrameObject::RelatedFrameInfo& info);

	private:
		std::weak_ptr<FrameObject> RelatedFramePtr;
	};
	
public:
/**
 * @brief Get the Frame ID
 * 
 * @return int Frame ID
 */
	int getID();


	/**
	 * @brief Get the timestamp.
	 * 
	 * @return uint32 timestamp
	 */
	uint32_t getTimestamp();

	/**
	 * @brief set the timestamp.
	 * @param time the new timestamp
	 */
	bool setTimestamp(uint32_t time);


	/**
	 * @brief set the best frame, the best camera pair is the camera that will be used during dual camera dense reconstruction.
	 * the best frame should be set after pose reconstruction in mono camera scenario, or be set when the frame is generated in dual camera or RGBD camera scenario.
	 * @param the shared pointer of best frame.
	 * @return wether created succeefully.
	 *
	 */
	bool setBestFrame(RelatedFrameInfo::Ptr FramePtr);

	/**
	 * @brief get the best frame.
	 * 
	 * @return the shared pointer of the best frame, the pointer will be empty when no frame exist.
	 */
	RelatedFrameInfo::Ptr getBestFrame();
	

	/**
	 * @brief add relate frame, the related frame is the frame with common field of view, 
	 * it may because the two frames have matched feature points, or because the two frames are created from an dual cameras, or RGBD camera.
	 * 
	 * @param FramePtr the shared pointer of related frame.
	 * @return true the frame is successfully created.
	 * @return false failed to create the frame, may because the related frame already exist.
	 */
	bool addRelatedFrame(RelatedFrameInfo::Ptr FramePtr);
	
	/**
	 * @brief remove the related camera frame, the related camera frame is the frame which has the common filed of view with current frame.
	 * 
	 * @param FrameID the ID of the frame to be removed
	 * @return true remove successfully
	 * @return false remove failed, the frame is not in the related camera frame list.
	 */
	bool removeRelatedFrame(int FrameID);

	/**
	 * @brief remove all of the related camera frame of current frame.
	 * 
	 * @return true remove successfully
	 * @return false remove failed, the current frame has no related camera frame.
	 */
	bool removeAllRelatedFrames();

	/**
	 * @brief update related frame.
	 * 
	 * @param FramePtr
	 * @return 
	 */
	bool updateRelatedFrame(RelatedFrameInfo::Ptr FramePtr);

	/**
	 * @brief Get the Related Frame object
	 * 
	 * @param FrameID frame ID
	 * @return FrameObject::Ptr the shared pointer of the frame
	 */
	RelatedFrameInfo::Ptr getRelatedFrame(int FrameID);

	/**
	 * @brief Get all the Related Frames object
	 * 
	 * @param Frames array of the shared pointer of the frame
	 * @return true has related frames
	 * @return false do NOT have related frames
	 */
	bool getAllRelatedFrames(std::set<RelatedFrameInfo::Ptr>& Frames);

	/**
	 * @brief get all the related frame id.
	 * 
	 * @param FrameID
	 * @return 
	 */
	bool getAllRelatedFrames(std::set<int>& FrameID);

	int hasRelatedFrame();

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
	 * @brief get mappoint id related to keypoint.
	 * 
	 * @param KeyPointID
	 * @return 
	 */
	bool getMapPoint(int KeyPointID, int& MappointID, Eigen::Vector4d& LocalCoordinate);

	/**
	 * @brief Get the Map Point object.
	 */
	std::shared_ptr<MapPointObject> getMapPoint(int KeyPointID);
	bool hasMappoint(int KeyPointID);

	/**
	 * @brief get all keypoint id related to mappoint.
	 * @param KeyPointID output set of keypoint ID.
	 * @return if get KeyPointID
	 */
	bool getAllMappointID(std::set<int>& KeyPointID);

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

	virtual bool load(JsonNode& fs) override;

	virtual bool save(JsonNode& fs) override;

	friend std::ostream& operator<<(std::ostream& os, FrameObject& frame);

	virtual std::string type_name() override;

	Sophus::SE3d getGlobalPose();
	void setGlobalPose(Sophus::SE3d& pose);
	Sophus::SE3d& GlobalPose();
	bool isGlobalPoseKnown();
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
	 * @brief this variable indicate which submap this frame belong.
	 */
	int MapID = -1;

private:
	/**
	 * @brief the global pose of current frame
	 *
	 */
	Sophus::SE3d m_GlobalPose;

	int ReferencedCount = 0;

protected:
	int FrameID;

	uint32_t Timestamp;
	/**
	 * @brief pointer of the related frame, the pose of related frame, the pose confidence of related frame (-1 means invalid)
	 */
	RelatedFrameInfo::Ptr BestCamera;

	/**
	 * @brief int is the frame ID, the second is the related frame pointer.
	 * the related frame pose might inaccurate once the global pose is found.
	 */
	std::map<int, RelatedFrameInfo::Ptr> RelatedFrame;

	using MapPointInfo = std::tuple<std::weak_ptr<MapPointObject>, Eigen::Vector4d, int>;
	std::map<int, MapPointInfo> ObservedMapPoints;

	bool KnownPose = false;
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
	static Ptr Create(int ID, int MapID, uint32_t Timestamp = 0);

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
		int MapID,
		uint32_t Timestamp = 0,
		const cv::Mat& RGBMat = cv::Mat(),
		const cv::Mat& XYZMat = cv::Mat());

public:
/**
 * @brief Construct a new PinholeFrameObject object
 * 
 * @param ID unique frame ID
 */
	PinholeFrameObject(int ID, uint32_t Timestamp = 0);

	/**
	 * @brief Destroy the Pinhole Frame Object object
	 * 
	 */
	~PinholeFrameObject();

	/**
	 * @brief 
	 * 
	 * @param fs 
	 * @return true 
	 * @return false 
	 * @bug the shared states between multiple same intrinsic matrix and distortion coefficient will lost.
	 */
	virtual bool load(JsonNode& fs) override;
	virtual bool save(JsonNode& fs) override;

	virtual std::string type_name() override;

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

