#pragma once
#include "WorkFlowObject.h"
#include "MVSConfig.h"
#include "FrameObject.h"
#include <opencv2/opencv.hpp>
#include <QObject>
#include <opencv2/features2d.hpp>

class FeatureExtractor : public WorkFlowObject
{
public:
	Q_OBJECT

public:
/**
 * @brief shared pointer type of FeatureExtractor
 */
	using Ptr = std::shared_ptr<FeatureExtractor>;
	
public:
/**
 * @brief Construct a new Feature Extractor object
 * 
 */
	FeatureExtractor();

	/**
	 * @brief Destroy the Feature Extractor object
	 * 
	 */
	virtual ~FeatureExtractor();
	
	/**
	 * @brief Get the Flow Name
	 * 
	 * @return std::string flow name
	 */
	virtual std::string type_name() override;

	bool clear() override;


	/**
	 * @brief detect feature points and compute the descriptors of the frame
	 * 
	 * @param frame 
	 * @return true detect success
	 * @return false detect failed
	 */
	virtual bool Compute(FrameObject::Ptr frame);

public slots:

/**
 * @warning **this method shold NOT be called since this algorithm can NOT process empty input data.**
 */
	void Trigger();

	void Trigger(DataQueue data) override;
	
protected:
	cv::Ptr<cv::Feature2D> detector;
};

class CVFeatureExtractor : public FeatureExtractor
{
public:
	Q_OBJECT
	using Ptr=std::shared_ptr<CVFeatureExtractor>;
public:
	static CVFeatureExtractor::Ptr Create();
public:
	CVFeatureExtractor();
	virtual ~CVFeatureExtractor();
	
	virtual bool load(JsonNode& fs) override;
	virtual bool save(JsonNode& fs) override;

	virtual std::string type_name() override;
};
