#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

class SemiGlobalMatching;
class SGMOption;

class CSGM : public cv::StereoMatcher
{
public:
	using Ptr = std::shared_ptr<CSGM>;
	static Ptr Create(int Width, int Height);
	static Ptr Create(int Width, int Height, SGMOption& option);
public:
	CSGM();
	CSGM(int Width, int Height);
	CSGM(int Width, int Height, SGMOption& option);
	virtual ~CSGM();
	
	void compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity) override;

	virtual int  getBlockSize() const override;
	virtual int  getDisp12MaxDiff() const override;
	virtual int  getMinDisparity() const override;
	virtual int  getNumDisparities() const override;
	virtual int  getSpeckleRange() const override;
	virtual int  getSpeckleWindowSize() const override;
	virtual void setBlockSize(int blockSize) override;
	virtual void setDisp12MaxDiff(int disp12MaxDiff) override;
	virtual void setMinDisparity(int minDisparity) override;
	virtual void setNumDisparities(int numDisparities) override;
	virtual void setSpeckleRange(int speckleRange) override;
	virtual void setSpeckleWindowSize(int speckleWindowSize) override;

	virtual void clear() override;
	virtual bool empty() const override;
	virtual cv::String getDefaultName() const override;

private:
	SemiGlobalMatching* m_pStereo;
	SGMOption* m_pOption;
};
