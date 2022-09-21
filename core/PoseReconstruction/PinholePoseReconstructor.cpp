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

bool PinholePoseReconstructor::SolveNewFramePose(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
	try 
	{
		std::set<int> relatedFrameID;
		if (!frame->getAllRelatedFrames(relatedFrameID)) return false;

		//如果全局姿态不是单位阵，说明姿态已知，这里就不求了
		if (frame->GlobalPose.matrix() != Eigen::Matrix4d::Identity())
			return true;

		auto result = this->SolveWithMappoint(frame, GlobalMap); //先用路点来求姿态

		//如果不存在路点，则使用对极几何来求姿态。使用对极几何来求位姿即重新新开地图初始化
		if (result == false)
		{
			//使用对极几何对每一个关联帧求相对姿态，并使用重投影误差最小的关联帧做初始值
			std::map<int, std::tuple<double, Sophus::SE3d>> Frameid2ErrNPose;
			for (auto& i : relatedFrameID)
			{
				/*
				只有关联帧没有关联帧的情况下才可以使用对极几何来初始化，
				关联帧如果有关联帧，说明该关联帧肯定存在路点，那么肯定属于某一个地图中，
				所以如果这里又用对极几何就会产生一个新的地图，将产生矛盾。
				该函数只考虑计算当前帧的位姿的情况，多个地图的拼接问题由其他函数考虑。
				*/
				std::set<int> RelatedRelatedFrameID; 
				if (frame->getRelatedFrame(i)->getRelatedFrame()->getAllRelatedFrames(RelatedRelatedFrameID))
					continue; //关联帧存在关联帧，不能用来初始化，跳过

				Sophus::SE3d tmp;
				double error = this->SolveWithEHMat(frame, frame->getRelatedFrame(i), tmp);
				if (error >= 0)
					Frameid2ErrNPose[i] = { error,tmp };
			}
			
			//assert(Frameid2ErrNPose.empty()); //应该不会都匹配错误，错误证明前节点有问题
			if (Frameid2ErrNPose.empty()) //没有合适的关联帧，那么这帧就无法使用，等待更多信息
				return false;
			
			//找到最小误差的ID
			int LeastErrorID = 0;
			double LeastError = DBL_MAX;
			for (auto& [key,value] : Frameid2ErrNPose)
			{
				if (std::get<double>(value) < LeastError)
				{
					LeastError = std::get<double>(value);
					LeastErrorID = key;
				}
			}
			
			//计算在这个子地图中的全局坐标
			frame->GlobalPose = std::get<Sophus::SE3d>(Frameid2ErrNPose[LeastErrorID]) 
				* frame->getRelatedFrame(LeastErrorID)->getRelatedFrame()->GlobalPose;
			
			//将该帧加入子地图
			frame->MapID = frame->getRelatedFrame(LeastErrorID)->getRelatedFrame()->MapID;
			GlobalMap->InitalFrames.at(frame->MapID) = LeastErrorID; //修改当前地图中的初始帧ID
			return true;
		}
		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	return false;
}

bool PinholePoseReconstructor::SolveWithMappoint(FrameObject::Ptr current, GlobalMapObject::Ptr GlobalMap)
{
	std::map<int, std::list<MapPointObject::Ptr>> MappointNumber; //用来统计每个小地图的关联路点数量，选择关联路点数量最多的小地图作为该相机的地图
	std::map< MapPointObject::Ptr, int> Keypoint2Mappoint; //特征点和路点的映射关系
	//融合地图的问题到下一步再考虑

	//从关联帧中提取出所有可以用于pnp的路点，以及将不同地图的路点进行分类
	std::set<int> relatedframes;
	current->getAllRelatedFrames(relatedframes);
	assert(!relatedframes.empty());
	for (auto& index : relatedframes)
	{
		//获得mapID
		//int MapID = GlobalMap->getFrameObjectMapID(index);
		auto RelatedFrameInfo = current->getRelatedFrame(index);
		auto relatedFramePtr = RelatedFrameInfo->getRelatedFrame();
		int MapID = relatedFramePtr->MapID;
		assert(MapID != -1);
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
	//int MapID = GlobalMap->getFrameObjectMapID(MappointNumber[LargestIndex].front()->getID());
	//GlobalMap->updateFrameObject(current, MapID);
	current->MapID = LargestIndex;
	return true;
}

double PinholePoseReconstructor::SolveWithEHMat(FrameObject::Ptr current, FrameObject::RelatedFrameInfo::Ptr referenced, Sophus::SE3d& T)
{
	if (referenced->KeyPointMatch.size() < 8) //八点法最少需要8个点
		return -1;
	
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

	DataFlowObject::cvMat2Sophus(finalT, T);
	return (E_Error > H_Error) ? H_Error : E_Error; //返回一个小的重投影误差
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

void PinholePoseReconstructor::ComputeMappoint(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
	//考虑计算新路点的问题和地图融合的问题

}
