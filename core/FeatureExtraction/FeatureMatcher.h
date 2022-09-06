#pragma once

#include "WorkFlowObject.h"
#include "MVSConfig.h"
#include <DBoW2/DBoW2.h>
#include <opencv2/opencv.hpp>
#include <QObject>
#include "GlobalMapObject.h"
#include "FrameObject.h"

/**
 * @brief The goal of this node is to select the keypoint pairs to match. 
 * The ambition is to find the images that are looking to the same areas of the scene.
 */
class FeatureMatcher:public WorkFlowObject
{
public:
	Q_OBJECT
    /**
     * @brief shared pointer type of FeatureMatcher
     */
	using Ptr = std::shared_ptr<FeatureMatcher>;

    /**
     * @brief create shared pointer of FeatureMatcher
     * 
     * @param GlobalMap the global map pointer
     * @return FeatureMatcher::Ptr shared pointer type of FeatureMatcher
     */
	FeatureMatcher::Ptr Create(GlobalMapObject::Ptr GlobalMap);

public:

/**
 * @brief Construct a new Feature Matcher object
 * 
 * @param GlobalMap  the global map pointer
 */
	FeatureMatcher(GlobalMapObject::Ptr GlobalMap);

    /**
     * @brief Destroy the Feature Matcher object
     * 
     */
	virtual ~FeatureMatcher();

/**
 * @brief Get the Flow Name
 * 
 * @return std::string flow name
 */
	std::string type_name() override;

    /**
     * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
     * 
     * @return true 
     * @return false 
     */
	virtual bool clear() override;

    /**
     * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
     * 
     * @param fs json file node
     * @return true 
     * @return false 
     */
	virtual bool init(JsonNode& fs) override;

    /**
     * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
     * 
     * @param fs json file node
     * @return true 
     * @return false 
     */
	virtual bool saveParameter(JsonNode& fs) override;

    /**
     * @brief match the features of the current frame with the global map
     * 
     * @param frame inputoutput frame object
     * @param GlobalMap the global map pointer, if the global map is empty,
     * the method will use preloaded global map
     * @return true match succeed
     * @return false match failed or input parameter incorrect
     */
	virtual bool Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap=GlobalMapObject::Ptr());

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

public slots:
/**
 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
 * **this method shold NOT be called since this algorithm can NOT process empty input data.**
 */
	void Trigger() override;

    /**
     * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
     * 
     * @param data 
     */
	void Trigger(DataQueue data) override;


protected:
    /**
     * @brief pointer of globalmap.
     */
	GlobalMapObject::Ptr GlobalMap;
};
