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

/**
 * @brief shared pointer type of FeatureExtractor
 */
	using Ptr = std::shared_ptr<FeatureExtractor>;

	/**
	 * @brief create shared pointer of FeatureExtractor with or without costomized detector type
	 * the customeized detector type is defined use OpenCV features2d framework, but is not implemented in OpenCV,
	 * so the method can not be initalized with the json file.
	 * 
	 * @param FeatureDetector the customized detector type
	 * @return FeatureExtractor::Ptr shared pointer type of FeatureExtractor
	 */
	static FeatureExtractor::Ptr Create(cv::Ptr<cv::Feature2D> FeatureDetector);
	
public:
/**
 * @brief Construct a new Feature Extractor object
 * 
 */
	FeatureExtractor();

	/**
	 * @brief Construct a new Feature Extractor object with customized detector type
	 * 
	 * @param FeatureDetector the customized detector type
	 */
	FeatureExtractor(cv::Ptr<cv::Feature2D> FeatureDetector);

	/**
	 * @brief Destroy the Feature Extractor object
	 * 
	 */
	~FeatureExtractor();
	
	/**
	 * @brief Get the Flow Name
	 * 
	 * @return std::string flow name
	 */
	std::string getFlowName() override;

	/**
	 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
	 * 
	 * @return true 
	 * @return false 
	 */
	
	bool clear() override;

	/**
	 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
	 * 
	 * @param fs 
	 * @return true 
	 * @return false 
	 */
	bool init(JsonNode& fs) override;

	/**
	 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
	 * 
	 * @param fs 
	 * @return true 
	 * @return false 
	 */
	bool saveParameter(JsonNode& fs) override;


	/**
	 * @brief detect feature points and compute the descriptors of the frame
	 * 
	 * @param frame 
	 * @return true detect success
	 * @return false detect failed
	 */
	bool Compute(FrameObject::Ptr frame);
	
public slots:
/**
 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
 * **this method shold NOT be called since this algorithm can NOT process empty input data.**
 * 
 */
	void Trigger();

	/**
	 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
	 * 
	 * @param data 
	 */
	void Trigger(DataQueue data) override;
	

	/**
 * @brief save data to file
 *
 * @param fs the json handler
 * @return true save successfully
 * @return false save failed
 */
	bool save(JsonNode& fs);

	/**
	 * @brief load data from file
	 *
	 * @param fs the json handler
	 * @return true load successfully
	 * @return false load failed
	 */
	bool load(JsonNode& fs);

private:
	cv::Ptr<cv::Feature2D> detector;
};

