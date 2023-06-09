#pragma once
#include "MVSConfig.h"
#include "DataFlowObject.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class FrameObject;

/**
 * @brief this is the map point class, this class store sparse 3D point.
 * 
 */
class MapPointObject : public DataFlowObject
{
public:
	using Ptr = std::shared_ptr<MapPointObject>;
	/**
	 * @brief 
	 * 
	 * @param ID 
	 * @return Ptr 
	 */
	static Ptr Create(int ID);

public:

	/**
	 * @brief Construct a new MapPointObject object.
	 * 
	 * @param ID
	 * @return 
	 */
	MapPointObject();
/**
 * @brief Construct a new MapPointObject object
 * 
 * @param ID MapPoint ID
 */
	MapPointObject(int ID);

	/**
	 * @brief Destroy the MapPointObject object
	 * 
	 */
	~MapPointObject();

public:
/**
 * @brief Get the Frame ID
 * 
 * @return int unique MapPoint ID
 */
	int getID();

/**
 * @brief Get the Observed Times
 * 
 * @return int the times of observation
 */
	int getObservedTimes();

/**
 * @brief add one observation
 * 
 * @param Frame the frame of observation
 * @param KeyPointID the keypoint id of observation in the frame
 * @return true add succeed
 * @return false add failed
 */
	bool addObservation(std::shared_ptr<FrameObject> Frame,int KeyPointID);

	/**
	 * @brief remove one observation
	 * 
	 * @param Frame the frame of observation
	 * @return true remove succeed
	 * @return false remove failed
	 */
	bool removeObservation(std::shared_ptr<FrameObject> Frame);

	/**
	 * @brief update the observation info
	 * 
	 * @param Frame the pointer of oervation frame
	 * @param KeyPointID the keypoint id of observation in the frame
	 * @return true update succeed
	 * @return false update failed, may be the observation is not exist, or input is invalid
	 */
	bool updateObservation(std::shared_ptr<FrameObject> Frame, int KeyPointID = -1);
	
	/**
	 * @brief Get the Observation object
	 * 
	 * @param FrameID FrameID
	 * @param Frame the shared pointer of frame
	 * @param KeyPointID the keypoint id of observation in the frame
	 * @return true get succeed
	 * @return false get failed, may be the observation is not exist
	 */
	bool getObservation(int FrameID, std::shared_ptr<FrameObject>& Frame,int &KeyPointID);

	/**
	 * @brief Get the All Observation object
	 * 
	 * @param Frames vector of pointers of frame 
	 * @return true get succeed
	 * @return false get failed
	 */
	bool getAllObservation(std::set<std::shared_ptr<FrameObject>>& Frames);

	/**
	 * @brief Get all the Observation frame id.
	 * 
	 * @param ids
	 * @return 
	 */
	bool getAllObservation(std::set<int>& ids);

/**
 * @brief Set the Map Point Quality object
 * 
 * @param quality the mappoint quality from 0 to 1
 * @return true set succeed
 * @return false set faled
 */
	bool setMapPointQuality(double quality);

	/**
	 * @brief Get the Map Point Quality object
	 * 
	 * @return double the map point quality from 0 to 1
	 */
	double getMapPointQuality();

	bool save(JsonNode& fs) override;

	bool load(JsonNode& fs) override;
	
	virtual std::string type_name() override;

	friend std::ostream& operator<<(std::ostream& os, MapPointObject& obj);

public:
/**
 * @brief the discriptor of map point related 2D key point
 */
	cv::Mat KeyPointDescriptor;

	/**
	 * @brief the global coordinate of the map point
	 */
	Eigen::Vector4d Position;

	/**
	 * @brief the observation direction of the map point
	 * 
	 */
	Eigen::Vector3d Normal;

private:
	int ID;

	/**
	 * the shared pointer of observerd frame, and the keypoint index.
	 * rememeber to delete the observation in related frame object when the mappoint is deleted!
	 */
	using ObservationInfo=std::tuple<std::weak_ptr<FrameObject>,int>;

	std::map<int,ObservationInfo> ObservedFrame;
	double quality;
};
