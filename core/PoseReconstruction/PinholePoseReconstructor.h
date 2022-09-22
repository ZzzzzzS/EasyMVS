#pragma once
#include "PoseReconstructor.h"

/**
 * @details this is the implementation of increasemental sfm(structure from motion)
 * under pinhole camera model secnario.
 */
class PinholePoseReconstructor: public PoseReconstructor
{
public:
	Q_OBJECT
	using Ptr = std::shared_ptr<PinholePoseReconstructor>;
	static PinholePoseReconstructor::Ptr Create(GlobalMapObject::Ptr GlobalMap);

public:
	PinholePoseReconstructor();
	PinholePoseReconstructor(GlobalMapObject::Ptr GlobalMap);
	virtual ~PinholePoseReconstructor();

	virtual bool save(JsonNode& fs) override;
	virtual bool load(JsonNode& fs) override;
	virtual bool clear() override;
	virtual bool Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap = GlobalMapObject::Ptr()) override;

private:
	bool SolveNewFramePose(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap);
	bool SolveWithMappoint(FrameObject::Ptr current, GlobalMapObject::Ptr GlobalMap);
	bool SolveWithMappoint(FrameObject::Ptr current, FrameObject::RelatedFrameInfo::Ptr related, GlobalMapObject::Ptr GlobalMap);
	double SolveWithEHMat(FrameObject::Ptr current, FrameObject::RelatedFrameInfo::Ptr referenced, Sophus::SE3d& T);
	double ComputeReprojectionError(std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dst,
		cv::Mat& CameraMat1, cv::Mat& CameraMat2, cv::Mat T);
	
	void ComputeMappoint(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap);
	void GenNewMappoints(FrameObject::Ptr frame, std::set<FrameObject::RelatedFrameInfo::Ptr>& Related, GlobalMapObject::Ptr GlobalMap);
	//void LocalOptimization();
	Sophus::SE3d SolvePNP(std::vector<cv::Point3d>& WorldPoints, std::vector<cv::Point2d>& CameraPoints, cv::Mat1d& Intrinsic);
};
