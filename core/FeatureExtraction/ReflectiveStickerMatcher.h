#pragma once
#include "FeatureMatcher.h"
#include "MVSConfig.h"
#include "GlobalMapObject.h"
#include "FrameObject.h"
#include <opencv2/opencv.hpp>
#include <QObject>

class ReflectiveStickerMatcher : public FeatureMatcher
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
	bool save(JsonNode& fs) override;
    bool load(JsonNode& fs) override;

	virtual bool Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap=GlobalMapObject::Ptr());

private:
	bool MatchWithKnownH(const std::vector<cv::KeyPoint>& Points1, const std::vector<cv::KeyPoint>& Point2,
		cv::InputArray H, std::vector<cv::DMatch>& Matches);
	void ComputeWithF(const std::vector<cv::KeyPoint>& Points1, const std::vector<cv::KeyPoint>& Points2, const cv::Mat1d& F, std::vector<cv::DMatch>& Matches);
};
