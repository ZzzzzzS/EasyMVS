#include "FeatureExtractor.h"


FeatureExtractor::Ptr FeatureExtractor::Create(cv::Ptr<cv::Feature2D> FeatureDetector)
{
		return std::make_shared<FeatureExtractor>(FeatureDetector);
}

FeatureExtractor::FeatureExtractor()
{
}

FeatureExtractor::FeatureExtractor(cv::Ptr<cv::Feature2D> FeatureDetector)
	:detector(FeatureDetector)
{
}

FeatureExtractor::~FeatureExtractor()
{
}

std::string FeatureExtractor::getFlowName()
{
	return std::string("Workflow FeatureExtractor");
}

bool FeatureExtractor::clear()
{
	return false;
}

bool FeatureExtractor::init(JsonNode& fs)
{
	return false;
}

bool FeatureExtractor::saveParameter(JsonNode& fs)
{
	return false;
}

bool FeatureExtractor::Compute(FrameObject::Ptr frame)
{
	return false;
}

void FeatureExtractor::Trigger(DataQueue data)
{
}

bool FeatureExtractor::save(JsonNode& fs)
{
	return false;
}

bool FeatureExtractor::load(JsonNode& fs)
{
	return false;
}

void FeatureExtractor::Trigger()
{
	emit this->Error("Can NOT triggered without input data");
}
