#include "PinholePoseReconstructor.h"

PinholePoseReconstructor::Ptr PinholePoseReconstructor::Create(GlobalMapObject::Ptr GlobalMap)
{
    return std::make_shared<PinholePoseReconstructor>(GlobalMap);
}

PinholePoseReconstructor::PinholePoseReconstructor()
{
}

PinholePoseReconstructor::PinholePoseReconstructor(GlobalMapObject::Ptr GlobalMap)
    :PoseReconstructor(GlobalMap)
{
}

PinholePoseReconstructor::~PinholePoseReconstructor()
{
}

bool PinholePoseReconstructor::save(JsonNode& fs)
{
    return false;
}

bool PinholePoseReconstructor::load(JsonNode& fs)
{
    return false;
}

bool PinholePoseReconstructor::clear()
{
    return false;
}

bool PinholePoseReconstructor::Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
	//计算新姿态
	//计算新路点（计算新路点+融合已知路点）
	//局部优化
    return false;
}

void PinholePoseReconstructor::SolveNewFramePose(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
	try
	{
		if (frame->getID() == GlobalMap->InitialFrameID) // do nothing in first frame
			return;

		std::set<int> relatedFrameID;
		if (!frame->getAllRelatedFrames(relatedFrameID)) return;

		this->SolveWithMappoint(frame); //先用路点来求姿态

		for (auto& i : relatedFrameID) //没办法了再用对极几何来求姿态
		{
			auto relatedFramePtr = frame->getRelatedFrame(i);
			
			if (relatedFramePtr->Pose != nullptr) continue; //位姿已知，不用求了
			
			if (this->SolveWithEHMat(frame, relatedFramePtr))
			{
				std::cout << this->type_name() << ": failed to solve pose between frame" <<
					frame->getID() << " and frame" << relatedFramePtr->getRelatedFrame()->getID() << std::endl;
			}
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	return;
}

bool PinholePoseReconstructor::SolveWithMappoint(FrameObject::Ptr current, GlobalMapObject::Ptr GlobalMap)
{
	std::map<int, std::list<MapPointObject::Ptr>> MappointNumber; //用来统计每个小地图的关联路点数量，选择关联路点数量最多的小地图作为该相机的地图
	std::map< MapPointObject::Ptr, int> Keypoint2Mappoint; //特征点和路点的映射关系
	//融合地图的问题到下一步再考虑

	//从关联帧中提取出所有可以用于pnp的路点，以及将不同地图的路点进行分类
	std::set<int> relatedframes;
	current->getAllRelatedFrames(relatedframes);
	for (auto& index : relatedframes)
	{
		//获得mapID
		int MapID = GlobalMap->getFrameObjectMapID(index);
		assert(MapID != -1);
		auto RelatedFrameInfo = current->getRelatedFrame(index);
		auto relatedFramePtr = RelatedFrameInfo->getRelatedFrame();
		std::set<int> RelateFrameMappoint;
		relatedFramePtr->getAllMappointID(RelateFrameMappoint);
		for (auto& dmatch : RelatedFrameInfo->KeyPointMatch)
		{
			//如果匹配的特征点有路点，那么就将这个点插入映射关系中，并加入关联路点表中
			if (RelateFrameMappoint.count(dmatch.trainIdx) == 1)
			{
				auto MappointPtr = relatedFramePtr->getMapPoint(dmatch.trainIdx);
				Keypoint2Mappoint.insert({ MappointPtr,dmatch.queryIdx });
				if (MappointNumber.count(MapID) == 0) //如果还不存在这个ID就先创建一个
				{
					MappointNumber.insert({ MapID,std::list<MapPointObject::Ptr>() });
				}
				MappointNumber[MapID].push_back(MappointPtr); //将路点加入到表中
			}
		}
	}

	//寻找关联路点最多的子地图
	int LargestIndex = 0;
	int LargestValue = 0;
	for (auto& [key,value] : MappointNumber)
	{
		if (value.size() > LargestValue)
		{
			LargestValue = value.size();
			LargestIndex = key;
		}
	}
	assert(LargestValue !=0);

	//找出对应的关联路点和特征点，转换数据格式，准备计算PnP
	std::vector<cv::Point3d> cvMapPoint;
	std::vector<cv::Point2d> cvKeyPoint;
	cvMapPoint.reserve(MappointNumber[LargestIndex].size());
	cvKeyPoint.reserve(MappointNumber[LargestIndex].size());
	for (auto& Mappoint : MappointNumber[LargestIndex])
	{
		cvMapPoint.emplace_back(Mappoint->Position(0),
			Mappoint->Position(1), Mappoint->Position(2));
		cvKeyPoint.emplace_back(current->KeyPoints.at((Keypoint2Mappoint.at(Mappoint))).pt);
	}

	if (cvMapPoint.size() < 4) //PnP最少需要3个点
		return false;

	cv::Mat1d K, rvec, R, t, T;
	K = std::dynamic_pointer_cast<PinholeFrameObject>(current)->CameraMatrix;
	cv::solvePnP(cvMapPoint, cvKeyPoint, K, cv::noArray(), R, t);
	R = cv::Mat1d::zeros(3, 3);
	cv::Rodrigues(rvec, R);
	T = DataFlowObject::Rt2T(R, t);
	Sophus::SE3d pose;
	DataFlowObject::cvMat2Sophus(T, pose);
	current->GlobalPose = pose;


	return true;
}

bool PinholePoseReconstructor::SolveWithEHMat(FrameObject::Ptr current, FrameObject::RelatedFrameInfo::Ptr referenced)
{
	if (referenced->KeyPointMatch.size() < 8) //八点法最少需要8个点
		return false;
	
	auto ReferencedFrame = std::dynamic_pointer_cast<PinholeFrameObject>(referenced->getRelatedFrame());
	auto PinholeCurrent = std::dynamic_pointer_cast<PinholeFrameObject>(current);
	
	std::vector<cv::Point2f> SrcPoints;
	std::vector<cv::Point2f> DstPoints;
	SrcPoints.reserve(referenced->KeyPointMatch.size());
	DstPoints.reserve(referenced->KeyPointMatch.size());
	for (auto& match : referenced->KeyPointMatch)
	{
		SrcPoints.push_back(PinholeCurrent->KeyPoints[match.queryIdx].pt);
		DstPoints.push_back(ReferencedFrame->KeyPoints[match.trainIdx].pt);
	}
	
	//TODO: 加一个从H矩阵恢复Rt的函数
	auto H_Mat = cv::findHomography(SrcPoints, DstPoints, cv::RANSAC);
	
	double H_Error = DBL_MAX; //HACK: 没有H到rt的函数，临时先这么干
	cv::Mat H_T;

	cv::Mat E_Mat, E_R, E_t, E_T;
	cv::recoverPose(SrcPoints, DstPoints, PinholeCurrent->CameraMatrix, cv::noArray(),
		ReferencedFrame->CameraMatrix, cv::noArray(), E_Mat, E_R, E_t, cv::RANSAC);
	E_T = DataFlowObject::Rt2T(E_R, E_T);
	double E_Error = this->ComputeReprojectionError(SrcPoints, DstPoints,
		PinholeCurrent->CameraMatrix, ReferencedFrame->CameraMatrix, E_T);
	
	cv::Mat1d finalT = (E_Error > H_Error) ? H_T : E_T;
	
	Sophus::SE3d SophusT;
	DataFlowObject::cvMat2Sophus(finalT, SophusT);
	referenced->Pose = std::make_shared<Sophus::SE3d>(SophusT);
	return true;
}

double PinholePoseReconstructor::ComputeReprojectionError(std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dst, cv::Mat& CameraMat1, cv::Mat& CameraMat2, cv::Mat T)
{
	cv::Mat1d K1_inv = cv::Mat1d::zeros(4, 3);
	K1_inv(3, 2) = 1;
	cv::Mat1d K1_invtmp = CameraMat1.inv();
	K1_invtmp.copyTo(K1_inv.colRange(0, 2).rowRange(0, 2));
	
	cv::Mat1d K2 = cv::Mat1d::zeros(3, 4);
	CameraMat2.copyTo(K2.colRange(0, 2).rowRange(0, 2));
	
	cv::Mat H_Mat = K2 * T * K1_inv;
	
	double Error = 0;
	for (size_t i = 0; i < src.size(); i++)
	{
		cv::Vec3f p1(src[i].x, src[i].y, 1);
		cv::Vec3f p2(dst[1].x, dst[i].y, 1);
		
		cv::Mat1d project = H_Mat * p1;
		
		double localerror = (p2(0) - project(0)) * (p2(0) - project(0)) +
			(p2(1) - project(1)) * (p2(1) - project(1));
		
		Error += localerror;
	}

	return Error;
}
