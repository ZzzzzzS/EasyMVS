#pragma once
#include "MVSConfig.h"
#include "WorkFlowObject.h"
#include "FrameObject.h"
#include "MapPointObject.h"
#include "GlobalMapObject.h"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <QObject>
#include <vector>
#include <queue>

class DenseReconstructor:public WorkFlowObject
{
public:
	Q_OBJECT
    /**
     * @brief the shared pointer type of DenseReconstructor
     * 
     */
	using Ptr = std::shared_ptr<DenseReconstructor>;

    /**
     * @brief create shared pointer of DenseReconstructor with or without customized matcher
     * 
     * @param CustomizeMatcher 
     * @return DenseReconstructor::Ptr 
     */
	static DenseReconstructor::Ptr Create(GlobalMapObject::Ptr GlobalMap);
	
public:

/**
 * @brief Construct a new Dense Reconstructor object with or without customized matcher
 * 
 * @param CustomizeMatcher 
 */
	DenseReconstructor(GlobalMapObject::Ptr GlobalMap);
	virtual ~DenseReconstructor();

/**
 * @brief Get the Flow Name object
 * This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
 * 
 * @return std::string 
 */
	std::string getFlowName() override;

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
     * @param fs 
     * @return true 
     * @return false 
     */
	virtual bool init(JsonNode& fs) override;

    /**
     * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
     * 
     * @param fs 
     * @return true 
     * @return false 
     */
	virtual bool saveParameter(JsonNode& fs) override;

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


    /**
     * @brief computes disparity map for the frame with preloaded parameters, and preloaded best camera(s).
     * 
     * @param frame the new comming frame
     * @return true compute success 
     * @return false compute failed
     */
    virtual bool Compute(FrameObject::Ptr frame);

    /**
     * @brief computes disparity map for the specified stereo pair with preloaded parameters,
     * the output disparity map will be stored in the frame.
     * 
     * @param frame1 the left frame
     * @param Frame2 the right frame
     * @return true compute success 
     * @return false compute failed, maybe the frame is not stereo pair
     */
	virtual bool Compute(FrameObject::Ptr frame1, FrameObject Frame2);

    /**
     * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
     * 
     * @param frames array of frames to compute disparity map
     * @return true compute success 
     * @return false compute failed, maybe the frame is not stereo pair
     */
	virtual bool Compute(std::vector<FrameObject::Ptr>& frames);
	
	
public slots:

/**
 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
 * 
 * @param data 
 */
	void Trigger(DataQueue data) override;

    /**
     * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
     * **this method shold NOT be called since this algorithm can NOT process empty input data.**
     */
	void Trigger() override;
	
	

protected:
	cv::Ptr<cv::StereoMatcher> MatcherPtr;
    GlobalMapObject::Ptr GlobalMap;
};

