#include "ADCWrapper.h"
#include "ADCensusStereo.h"
#include "adcensus_types.h"

StereoADSensus::Ptr StereoADSensus::Create(int Width, int Height, ADCensusOption& option)
{
	return std::make_shared<StereoADSensus>(Width, Height, option);
}

StereoADSensus::Ptr StereoADSensus::Create(int Width, int Height)
{
	return std::make_shared<StereoADSensus>(Width, Height);
}

StereoADSensus::StereoADSensus()
	:m_pStereo(nullptr),
	m_pOption(nullptr)
{
}

StereoADSensus::StereoADSensus(int Width, int Height, ADCensusOption& option)
{
	m_pStereo = new ADCensusStereo();
	this->m_pOption = new ADCensusOption(option);
	if(!m_pStereo->Initialize(Width, Height, option))
		throw std::exception("failed to initialize adcensus\n");
}

StereoADSensus::StereoADSensus(int Width, int Height)
{
	this->m_pOption = new ADCensusOption();
	this->m_pStereo = new ADCensusStereo();
	if (!this->m_pStereo->Initialize(Width, Height, *m_pOption))
		throw std::exception("failed to initialize adcensus\n");
}

StereoADSensus::~StereoADSensus()
{
	if (this->m_pOption != nullptr)
		delete this->m_pOption;
	if (this->m_pStereo != nullptr)
		delete this->m_pStereo;
}

void StereoADSensus::compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
	cv::Mat left_ = left.getMat();
	cv::Mat right_ = right.getMat();
	disparity.create(left_.size(), CV_32FC1);
	cv::Mat disparity_ = disparity.getMat();
	if (left_.channels() != 3 || right_.channels() != 3)
		throw std::exception("only support rgb image\n");
	if (this->m_pStereo != nullptr)
	{
		this->m_pStereo->Match(left_.data, right_.data, (float*)disparity_.data);
	}
}

int StereoADSensus::getBlockSize() const
{
	return 0;
}

int StereoADSensus::getDisp12MaxDiff() const
{
	return 0;
}

int StereoADSensus::getMinDisparity() const
{
	return 0;
}

int StereoADSensus::getNumDisparities() const
{
	return 0;
}

int StereoADSensus::getSpeckleRange() const
{
	return 0;
}

int StereoADSensus::getSpeckleWindowSize() const
{
	return 0;
}

void StereoADSensus::setBlockSize(int blockSize)
{
}

void StereoADSensus::setDisp12MaxDiff(int disp12MaxDiff)
{
}

void StereoADSensus::setMinDisparity(int minDisparity)
{
}

void StereoADSensus::setNumDisparities(int numDisparities)
{
}

void StereoADSensus::setSpeckleRange(int speckleRange)
{
}

void StereoADSensus::setSpeckleWindowSize(int speckleWindowSize)
{
}

void StereoADSensus::clear()
{
	if (this->m_pStereo != nullptr)
		delete this->m_pStereo;
}

bool StereoADSensus::empty() const
{
	return (this->m_pStereo == nullptr) ? true : false;
}

cv::String StereoADSensus::getDefaultName() const
{
	return std::string("ad-sensus");
}
