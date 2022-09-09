#include "FeatureExtractor.h"

FeatureExtractor::FeatureExtractor()
{
}

FeatureExtractor::~FeatureExtractor()
{
}

std::string FeatureExtractor::type_name()
{
	return std::string("Workflow FeatureExtractor");
}

bool FeatureExtractor::clear()
{
	this->detector->clear();
	this->detector.reset();
	this->m_isInit=false;
	return true;
}

bool FeatureExtractor::Compute(FrameObject::Ptr frame)
{
	cv::Mat rgb=frame->RGBMat;
	this->detector->detectAndCompute(frame->RGBMat,cv::noArray(),frame->KeyPoints,frame->KeyPointsDescriptors);
	return !frame->KeyPoints.empty();
}

void FeatureExtractor::Trigger(DataQueue data)
{
	DataQueue DataOut;
	while (!data.empty())
	{
		auto ptr=std::dynamic_pointer_cast<FrameObject>(data.front());
		if(ptr==nullptr)
		{
			emit this->Warning(this->type_name()+": failed to dynamic cast input data");
		}
		else
		{
			this->Compute(ptr);
		}
		data.pop();
		DataOut.push(ptr);
	}
	emit this->Finished(DataOut);
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

CVFeatureExtractor::Ptr CVFeatureExtractor::Create() 
{
	return std::make_shared<CVFeatureExtractor>();
}

CVFeatureExtractor::CVFeatureExtractor() 
{
	this->detector=cv::ORB::create();
	//HACK: 
}

CVFeatureExtractor::~CVFeatureExtractor() 
{
}

bool CVFeatureExtractor::init(JsonNode& fs) 
{
	this->m_isInit=true;
	return true;
}

bool CVFeatureExtractor::saveParameter(JsonNode& fs) 
{
	return false;
}
