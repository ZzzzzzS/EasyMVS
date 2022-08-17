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
 * @brief Construct a new MapPointObject object
 * 
 * @param ID 
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
	int getFrameID();

public:
	cv::Mat KeyPointDescriptor;
	Eigen::Vector4d KeyPoint;
	Eigen::Vector3d Normal;
	double quality;

private:
	const int ID;
	using ObservedFrameInfo = std::tuple<std::weak_ptr<FrameObject>, int>;
	std::map<int, ObservedFrameInfo> ObservedFrame;

};
