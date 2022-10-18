#pragma once
#include "FeatureMatcher.h"
#include "MVSConfig.h"
#include "GlobalMapObject.h"
#include "FrameObject.h"
#include "VocTreeMatcher.h"
#include <opencv2/opencv.hpp>
#include <QObject>

class ReflectiveStickerMatcher : public VocTreeMatcher
{
public:
	Q_OBJECT
public:
	using Ptr = std::shared_ptr<ReflectiveStickerMatcher>;
	static Ptr Create(GlobalMapObject::Ptr GlobalMap);
public:
	ReflectiveStickerMatcher(GlobalMapObject::Ptr GlobalMap);
	virtual ~ReflectiveStickerMatcher();

	std::string type_name() override;
	virtual bool clear() override;
	virtual bool save(JsonNode& fs) override;
    virtual bool load(JsonNode& fs) override;

	virtual bool Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap=GlobalMapObject::Ptr());

private:
	void ComputeWithF(const std::vector<cv::KeyPoint>& Points1, const std::vector<cv::KeyPoint>& Points2, const cv::Mat1d& F, std::vector<cv::DMatch>& Matches);
	bool MatchAndFindF(FrameObject::Ptr frame1, FrameObject::Ptr frame2, cv::Mat1d& F);
private:
	FrameObject::Ptr LastFrame;
	//这没用DBoW回环检测了，就简单使用帧序号来判断是否闭环了
	int TotalFrameNumber;
	int CurrentFrameIndex;
};
