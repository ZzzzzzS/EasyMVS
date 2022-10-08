#include "CensusSGMWrapper.h"
#include "SemiGlobalMatching.h"
#include "sgm_types.h"

CSGM::Ptr CSGM::Create(int Width, int Height)
{
    return std::make_shared<CSGM>(Width, Height);
}

CSGM::Ptr CSGM::Create(int Width, int Height, SGMOption& option)
{
    return std::make_shared<CSGM>(Width, Height, option);
}

CSGM::CSGM()
    :m_pStereo(nullptr),
    m_pOption(nullptr)
{
}

CSGM::CSGM(int Width, int Height)
{
	this->m_pOption = new SGMOption();
	this->m_pStereo = new SemiGlobalMatching();
	if (!this->m_pStereo->Initialize(Width, Height, *m_pOption))
		throw std::exception("failed to initialize sgm\n");
}

CSGM::CSGM(int Width, int Height, SGMOption& option)
{
	this->m_pStereo = new SemiGlobalMatching();
	this->m_pOption = new SGMOption(option);
	if (!this->m_pStereo->Initialize(Width, Height, option))
		throw std::exception("failed to initialize sgm\n");
}

CSGM::~CSGM()
{
	if (this->m_pOption != nullptr)
		delete this->m_pOption;
	if (this->m_pStereo != nullptr)
		delete this->m_pStereo;
}

void CSGM::compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
	cv::Mat left_ = left.getMat();
	cv::Mat right_ = right.getMat();
	
	disparity.create(left_.size(), CV_32FC1);
	cv::Mat disparity_ = disparity.getMat();
	if (left_.channels() != 1 || right_.channels() != 1)
	{
		//throw std::exception("only support gray image\n");
		if (left_.type() == CV_8UC3)
		{
			cv::cvtColor(left_, left_, cv::COLOR_BGR2GRAY);
		}
		else
		{
			throw std::exception("only support gray image\n");
		}
		
		if (right_.type() == CV_8UC3)
		{
			cv::cvtColor(right_, right_, cv::COLOR_BGR2GRAY);
		}
		else
		{
			throw std::exception("only support gray image\n");
		}
	}
		
	
	if (this->m_pStereo != nullptr)
	{
		this->m_pStereo->Match(left_.data, right_.data, (float*)disparity_.data);
	}
}

int CSGM::getBlockSize() const
{
	return this->m_pOption->census_size;
}

int CSGM::getDisp12MaxDiff() const
{
	return 0;
}

int CSGM::getMinDisparity() const
{
    return 0;
}

int CSGM::getNumDisparities() const
{
    return 0;
}

int CSGM::getSpeckleRange() const
{
    return 0;
}

int CSGM::getSpeckleWindowSize() const
{
    return 0;
}

void CSGM::setBlockSize(int blockSize)
{
}

void CSGM::setDisp12MaxDiff(int disp12MaxDiff)
{
}

void CSGM::setMinDisparity(int minDisparity)
{
}

void CSGM::setNumDisparities(int numDisparities)
{
}

void CSGM::setSpeckleRange(int speckleRange)
{
}

void CSGM::setSpeckleWindowSize(int speckleWindowSize)
{
}

void CSGM::clear()
{
	if (this->m_pStereo != nullptr)
		delete this->m_pStereo;
}

bool CSGM::empty() const
{
	return (this->m_pStereo == nullptr) ? true : false;
}

cv::String CSGM::getDefaultName() const
{
	return cv::String("census-sgm");
}
