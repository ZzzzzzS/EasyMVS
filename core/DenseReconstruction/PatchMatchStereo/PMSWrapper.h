#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

class PatchMatchStereo;
struct PMSOption;

class StereoPMS : public cv::StereoMatcher
{
public:
	using Ptr = std::shared_ptr<StereoPMS>;
	static Ptr Create(int Width, int Height);
	static Ptr Create(int Width, int Height, const PMSOption& option);
	
public:
	StereoPMS();
	StereoPMS(int Width, int Height);
	StereoPMS(int Width, int Height, const PMSOption& option);
	~StereoPMS();
	virtual void compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity) override;
	
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

	virtual bool empty() const override;
	virtual void clear() override;
	virtual cv::String getDefaultName() const override;
private:
	PatchMatchStereo* m_pStereo;
	PMSOption* m_pOption;
};
