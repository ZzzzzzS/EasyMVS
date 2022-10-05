#include "PMSWrapper.h"
#include "pms_types.h"
#include "PatchMatchStereo.h"

StereoPMS::Ptr StereoPMS::Create(int Width, int Height)
{
	return std::make_shared<StereoPMS>(Width, Height);
}

StereoPMS::Ptr StereoPMS::Create(int Width, int Height, const PMSOption& option)
{
	return std::make_shared<StereoPMS>(Width, Height, option);
}

StereoPMS::StereoPMS()
	:m_pOption(nullptr),
	m_pStereo(nullptr)
{
}

StereoPMS::StereoPMS(int Width, int Height)
{
	this->m_pOption = new PMSOption();
	this->m_pStereo = new PatchMatchStereo();
	this->m_pStereo->Initialize((sint32)Width, (sint32)Height, *this->m_pOption);
}

StereoPMS::StereoPMS(int Width, int Height, const PMSOption& option)
{
	this->m_pOption = new PMSOption(option);
	this->m_pStereo = new PatchMatchStereo();
	this->m_pStereo->Initialize((sint32)Width, (sint32)Height, *this->m_pOption);
}

StereoPMS::~StereoPMS()
{
	if (this->m_pStereo != nullptr)
		delete this->m_pStereo;
	if (this->m_pOption != nullptr)
		delete this->m_pOption;
}

void StereoPMS::compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
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

int StereoPMS::getBlockSize() const
{
	return 0;
}

int StereoPMS::getDisp12MaxDiff() const
{
	return 0;
}

int StereoPMS::getMinDisparity() const
{
	return 0;
}

int StereoPMS::getNumDisparities() const
{
	return 0;
}

int StereoPMS::getSpeckleRange() const
{
	return 0;
}

int StereoPMS::getSpeckleWindowSize() const
{
	return 0;
}

void StereoPMS::setBlockSize(int blockSize)
{
}

void StereoPMS::setDisp12MaxDiff(int disp12MaxDiff)
{
}

void StereoPMS::setMinDisparity(int minDisparity)
{
}

void StereoPMS::setNumDisparities(int numDisparities)
{
}

void StereoPMS::setSpeckleRange(int speckleRange)
{
}

void StereoPMS::setSpeckleWindowSize(int speckleWindowSize)
{
}

bool StereoPMS::empty() const
{
	return (this->m_pStereo == nullptr) ? true : false;
}

void StereoPMS::clear()
{
	if (this->m_pStereo != nullptr)
		delete this->m_pStereo;
}

cv::String StereoPMS::getDefaultName() const
{
	return cv::String("patch-match-stereo");
}
