#pragma once
#include "PoseReconstructor.h"
#include <QObject>

class ReflectiveReconstructor : public PoseReconstructor
{
public:
	Q_OBJECT
public:
	using Ptr= std::shared_ptr<ReflectiveReconstructor>;
	static ReflectiveReconstructor::Ptr Create(GlobalMapObject::Ptr GlobalMap);
	ReflectiveReconstructor(GlobalMapObject::Ptr GlobalMap);
	virtual ~ReflectiveReconstructor();
public:
	virtual bool Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap = GlobalMapObject::Ptr()) override;
	
	std::string type_name() override;
	virtual bool save(JsonNode& fs) override;
	virtual bool load(JsonNode& fs) override;
	virtual bool clear() override;
private:
	void GenNewMappoint(FrameObject::Ptr frame, FrameObject::RelatedFrameInfo::Ptr related, GlobalMapObject::Ptr GlobalMap);
	bool MergeMap(FrameObject::Ptr frame, FrameObject::RelatedFrameInfo::Ptr related, GlobalMapObject::Ptr GlobalMap);
	Sophus::SE3d SolvePNP(std::vector<cv::Point3d>& WorldPoints, std::vector<cv::Point2d>& CameraPoints, cv::Mat1d& Intrinsic);
};
