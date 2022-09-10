#pragma once
#include "MVSConfig.h"
#include "DataFlowObject.h"
#include "FrameObject.h"
#include "MapPointObject.h"
#include "DenseMapObject.h"

class GlobalMapObject: public DataFlowObject
{
public:
/**
 * @brief shared pointer of GlobalMapObject
 * 
 */
	using Ptr = std::shared_ptr<GlobalMapObject>;

    /**
     * @brief create shared pointer of GlobalMapObject
     * 
     * @return GlobalMapObject::Ptr 
     */
	static GlobalMapObject::Ptr Create();
public:

/**
 * @brief Construct a new Global Map Object object
 * 
 */
	GlobalMapObject();

    /**
     * @brief Destroy the Global Map Object object
     * 
     */
	~GlobalMapObject();
	
    /**
     * @brief add frame to global map.
     */
    bool addFrameObject(FrameObject::Ptr frame);

    /**
     * @brief add map point to global map.
     */
    bool addMapPoint(MapPointObject::Ptr mappoint);

    /**
     * @brief map contain frames, Frame ID and Frame pointer
     * 
     */
	std::map<int, FrameObject::Ptr> Frames;

    /**
     * @brief map contain map points, MapPoint ID and MapPoint pointer
     * 
     */
	std::map<int, MapPointObject::Ptr> MapPoints;

    /**
     * @brief the initial frame ID
     */
    int InitialFrameID;

    /**
     * @brief get all related frame as a whole map, return different map frames, and Map Points.
     * 
     * @param Frames a set of maps' frames.
     * @param MapPoint a set of maps' Mappoints.
     * @return whether can find map.
     */
    bool getMaps(std::set<std::map<int, FrameObject>>& Frames, std::set<std::map<int, MapPointObject>>& MapPoint);

    bool save(JsonNode& fs) override;

    bool load(JsonNode& fs) override;

    virtual std::string type_name() override;

private:

};

