#pragma once

#include "WorkFlowObject.h"
#include "MVSConfig.h"
#include <DBoW2/DBoW2.h>
#include <opencv2/opencv.hpp>
#include <QObject>
#include "GlobalMapObject.h"
#include "FrameObject.h"

/**
 * @details The goal of this node is to select the keypoint pairs to match. 
 * The ambition is to find the images that are looking to the same areas of the scene.
 */
class FeatureMatcher:public WorkFlowObject
{
public:
	Q_OBJECT
public:
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

	std::string type_name() override;

	virtual bool clear() override;

    /**
     * @brief match the features of the current frame within the global map
     * 
     * @param frame inputoutput frame object
     * @param GlobalMap the global map pointer, if the global map is empty,
     * the method will use preloaded global map
     * @return true match succeed
     * @return false match failed or input parameter incorrect
     */
	virtual bool Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap=GlobalMapObject::Ptr());

    /**
     * @brief find related frames of input frame within the global map.
     * 
	 * @param frame input frame object
	 * @param related output related frame
	 * @param the global map pointer, if the global map is empty,
     * the method will use preloaded global map
     * @ return if related frame found
     */
    virtual bool MatchRelatedFrame(FrameObject::Ptr frame, std::list<FrameObject::Ptr>& related, GlobalMapObject::Ptr GlobalMap = GlobalMapObject::Ptr());
	

	/**
	 * @brief find matched keypoints between the input frame and its related frame.
	 * 
	 * @param frame inputoutput frame object
	 * @return if matched keypoints found
	 */
    virtual bool MatchKeyPoints(FrameObject::Ptr frame);

    /**
     * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
     * 
     * @param queryFrames query frame
     * @param trainFrames train frame
     * @param matches keypoint match info
     * @return if related frame found
     */
    virtual bool MatchKeyPoints(FrameObject::Ptr queryFrames, FrameObject::Ptr trainFrames, std::vector<cv::DMatch>& matches);

    bool save(JsonNode& fs) override;

    bool load(JsonNode& fs) override;

public slots:
/**
 * @warning **this method shold NOT be called since this algorithm can NOT process empty input data.**
 */
	void Trigger() override;

	void Trigger(DataQueue data) override;


protected:
    /**
     * @brief pointer of globalmap.
     */
	GlobalMapObject::Ptr GlobalMap;
};
