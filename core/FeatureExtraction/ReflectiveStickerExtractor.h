#pragma once
#include "FeatureExtractor.h"
#include "MVSConfig.h"
#include "FrameObject.h"
#include <opencv2/opencv.hpp>
#include <QObject>

class ReflectiveStickerExtractor : public CVFeatureExtractor
{
public:
	Q_OBJECT
public:
	using Ptr = std::shared_ptr<ReflectiveStickerExtractor>;
	static Ptr Create();
		
public:
	ReflectiveStickerExtractor();
	virtual ~ReflectiveStickerExtractor();

	virtual bool Compute(FrameObject::Ptr frame) override;
	
	virtual bool load(JsonNode& fs) override;
	virtual bool save(JsonNode& fs) override;

	virtual std::string type_name() override;

	bool clear() override;
	
private:
	void ComputeReflectiveSticker(cv::Mat& img,std::vector<cv::KeyPoint>& KeyPoints);
};

