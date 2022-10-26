#include "ReflectiveReconstructor.h"

ReflectiveReconstructor::Ptr ReflectiveReconstructor::Create(GlobalMapObject::Ptr GlobalMap)
{
	return std::make_shared<ReflectiveReconstructor>(GlobalMap);
}

ReflectiveReconstructor::ReflectiveReconstructor(GlobalMapObject::Ptr GlobalMap)
	:PoseReconstructor(GlobalMap)
{
	std::cout << "Welcome to use ReflectiveReconstructor" << std::endl;
}

ReflectiveReconstructor::~ReflectiveReconstructor()
{
}

std::string ReflectiveReconstructor::type_name()
{
	return std::string("Workflow-Pose-Reconstructor-Reflective");
}

bool ReflectiveReconstructor::Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap_)
{
	GlobalMapObject::Ptr GlobalMap = (GlobalMap_ == nullptr) ? this->GlobalMap : GlobalMap_;
	std::vector<FrameObject::RelatedFrameInfo::Ptr> UnKnownPoseFrame;
	FrameObject::RelatedFrameInfo::Ptr CurrentFrameInfo;
	
	std::set<int> RelatedFramesID;
	frame->getAllRelatedFrames(RelatedFramesID);
	for (auto& id : RelatedFramesID)
	{
		auto related = frame->getRelatedFrame(id);
		if (related->getRelatedFrame()->MapID!=frame->MapID)
		{
			UnKnownPoseFrame.push_back(related);
		}
		else
		{
			CurrentFrameInfo = related;
		}
	}

	this->GenNewMappoint(frame, CurrentFrameInfo, GlobalMap);
	
	for (auto& RelatedInfo : UnKnownPoseFrame)
	{
		auto result = this->MergeMap(frame, RelatedInfo, GlobalMap);
		if (!result)
		{
			std::cout << this->type_name() << 
				": failed to mearge two maps, between map "
				<< frame->MapID << " and map " 
				<< RelatedInfo->getRelatedFrame()->MapID << std::endl;
		}
	}
	
	return true;
}
bool ReflectiveReconstructor::save(JsonNode& fs)
{
	return true;
}

bool ReflectiveReconstructor::load(JsonNode& fs)
{
	return true;
}

bool ReflectiveReconstructor::clear()
{
	return true;
}

void ReflectiveReconstructor::GenNewMappoint(FrameObject::Ptr frame, FrameObject::RelatedFrameInfo::Ptr related, GlobalMapObject::Ptr GlobalMap)
{
	auto PinholeFrame1 = std::dynamic_pointer_cast<PinholeFrameObject>(frame);
	auto PinholeFrame2 = std::dynamic_pointer_cast<PinholeFrameObject>(related->getRelatedFrame());
	
	cv::Mat1d T1, T2;
	DataFlowObject::Sophus2cvMat(PinholeFrame1->getGlobalPose().inverse(), T1);
	DataFlowObject::Sophus2cvMat(PinholeFrame2->getGlobalPose().inverse(), T2);
	
	cv::Mat1d projMatr1 = DataFlowObject::CameraMatrix3x4(PinholeFrame1->CameraMatrix) * T1;
	cv::Mat1d projMatr2 = DataFlowObject::CameraMatrix3x4(PinholeFrame2->CameraMatrix) * T2;
	cv::Mat1d projPoints1(2, related->KeyPointMatch.size());
	cv::Mat1d projPoints2(2, related->KeyPointMatch.size());
	cv::Mat1d projPoints3D;

	int count = 0;
	for (auto&& item : related->KeyPointMatch)
	{
		projPoints1(0, count) = PinholeFrame1->KeyPoints[item.queryIdx].pt.x;
		projPoints1(1, count) = PinholeFrame1->KeyPoints[item.queryIdx].pt.y;
		projPoints2(0, count) = PinholeFrame2->KeyPoints[item.trainIdx].pt.x;
		projPoints2(1, count) = PinholeFrame2->KeyPoints[item.trainIdx].pt.y;
		count++;
	}

	cv::triangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2, projPoints3D);
	for (size_t i = 0; i < projPoints3D.cols; i++)
	{
		cv::Mat1d point3D = projPoints3D.col(i);
		point3D /= point3D(3, 0);
		auto Mappoint = MapPointObject::Create(GlobalMap->AssignMappointID());
		Mappoint->Position = Eigen::Vector4d(point3D(0), point3D(1), point3D(2), point3D(3));
		Mappoint->addObservation(PinholeFrame1, related->KeyPointMatch[i].queryIdx);
		Mappoint->addObservation(PinholeFrame2, related->KeyPointMatch[i].trainIdx);
		PinholeFrame1->addMapPoint(related->KeyPointMatch[i].queryIdx,
			Mappoint, PinholeFrame1->getGlobalPose().inverse() * Mappoint->Position);
		PinholeFrame2->addMapPoint(related->KeyPointMatch[i].trainIdx,
			Mappoint, PinholeFrame2->getGlobalPose().inverse() * Mappoint->Position);
		GlobalMap->addMapPoint(Mappoint);
	}

	/** test */
	cv::Mat1d point3D1 = projMatr1 * projPoints3D;
	cv::Mat1d point3D2 = projMatr2 * projPoints3D;
	for (size_t i = 0; i < projPoints3D.cols; i++)
	{
		point3D1.col(i) /= point3D1(2, i);
		point3D2.col(i) /= point3D2(2, i);
	}
	__nop();
}


bool ReflectiveReconstructor::MergeMap(FrameObject::Ptr current, FrameObject::RelatedFrameInfo::Ptr related, GlobalMapObject::Ptr GlobalMap)
{
	auto referencePtr = related->getRelatedFrame();
	std::vector<cv::Point3d> cvMapPoint;
	std::vector<cv::Point2d> cvKeyPoint;
	
	//<老地图的路点,新地图的路点>
	std::list<std::tuple<MapPointObject::Ptr, MapPointObject::Ptr>> MapPointMatch;
	
	std::vector<cv::DMatch> MappointMatch;

	for (auto& item : related->KeyPointMatch)
	{
		if (referencePtr->hasMappoint(item.trainIdx))
		{
			MappointMatch.push_back(item);
			auto MapPoint = referencePtr->getMapPoint(item.trainIdx)->Position;
			auto KeyPoint = current->KeyPoints.at(item.queryIdx).pt;
			cvMapPoint.emplace_back(MapPoint(0), MapPoint(1), MapPoint(2));
			cvKeyPoint.emplace_back(KeyPoint);
			if (current->hasMappoint(item.queryIdx))
			{
				MapPointMatch.emplace_back(std::make_tuple(referencePtr->getMapPoint(item.trainIdx), current->getMapPoint(item.queryIdx)));
			}
		}
	}

	cv::Mat look;
	cv::drawMatches(current->RGBMat, current->KeyPoints, referencePtr->RGBMat, referencePtr->KeyPoints, MappointMatch, look);
	
	if (cvMapPoint.size() < 4) //PNP最低点数要求
		return false;

	auto K = std::dynamic_pointer_cast<PinholeFrameObject>(current)->CameraMatrix;
	auto pose = this->SolvePNP(cvMapPoint, cvKeyPoint, K);
	
	auto CurrentFrameCurrentPose = current->getGlobalPose();
	auto PoseTrans = pose * (CurrentFrameCurrentPose.inverse());
	

	/**ID的合并********/
	std::set<int> relatedID;
	current->getAllRelatedFrames(relatedID);
	for (auto&& id : relatedID)
	{
		auto RelatedFramePtr = current->getRelatedFrame(id);
		if (RelatedFramePtr->getRelatedFrame()->MapID == current->MapID)
		{
			auto FramePtr = RelatedFramePtr->getRelatedFrame();
			auto FramePose = FramePtr->getGlobalPose();
			FramePtr->setGlobalPose(PoseTrans * FramePose);
			FramePtr->MapID = referencePtr->MapID;
		}
	}
	current->setGlobalPose(pose);
	current->MapID = referencePtr->MapID;
	/**路点的合并************************/
	for (auto& [OldMappoint, NewMappoint] : MapPointMatch)
	{
		std::set<int> frameID;
		NewMappoint->getAllObservation(frameID);
		for (auto&& id : frameID)
		{
			FrameObject::Ptr framePtr;
			int keypointID;
			NewMappoint->getObservation(id, framePtr, keypointID);
			
			NewMappoint->removeObservation(framePtr);
			OldMappoint->addObservation(framePtr, keypointID);
			
			framePtr->updateMapPoint(keypointID, OldMappoint,
				framePtr->getGlobalPose().inverse() * OldMappoint->Position);
		}
		GlobalMap->removeMapPoint(NewMappoint->getID());
		std::cout << "NEW POINT" << NewMappoint.use_count() << std::endl;
		std::cout << "OLD POINT" << OldMappoint.use_count() << std::endl;
	}
	return true;
}

Sophus::SE3d ReflectiveReconstructor::SolvePNP(std::vector<cv::Point3d>& WorldPoints, std::vector<cv::Point2d>& CameraPoints, cv::Mat1d& Intrinsic)
{
	cv::Mat1d rvec, R, t, T;
	cv::solvePnP(WorldPoints, CameraPoints, Intrinsic, cv::noArray(), rvec, t,false,cv::SOLVEPNP_EPNP);
	R = cv::Mat1d::zeros(3, 3);
	cv::Rodrigues(rvec, R);
	T = DataFlowObject::Rt2T(R, t);
	Sophus::SE3d pose;
	DataFlowObject::cvMat2Sophus(T, pose);
	return pose;
}