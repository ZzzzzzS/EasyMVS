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
     * @brief map contain frames, Frame ID and Frame pointer
     * 
     */
	std::map<int, FrameObject::Ptr> Frames;

    /**
     * @brief map contain map points, MapPoint ID and MapPoint pointer
     * 
     */
	std::map<int, MapPointObject::Ptr> MapPoints;

private:

};

