#include "PinholePoseReconstructor.h"
#include <opencv2/core/eigen.hpp>

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
    return true;
}

bool PinholePoseReconstructor::load(JsonNode& fs)
{
    return true;
}

bool PinholePoseReconstructor::clear()
{
    return true;
}

bool PinholePoseReconstructor::Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
	//计算新姿态
	//计算新路点（计算新路点+融合已知路点）
	//局部优化
	if (GlobalMap == nullptr)
		GlobalMap = this->GlobalMap;

	auto result1 = this->SolveNewFramePose(frame, GlobalMap);
	if (!result1)
	{
		std::cout << this->type_name() << ": failed to solve new frame pose, frame ID=" << frame->getID() << std::endl;
		return false;
	}
	
	this->ComputeMappoint(frame, GlobalMap);
	return true;
}

bool PinholePoseReconstructor::SolveNewFramePose(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
	try 
	{
		std::set<int> relatedFrameID;
		//找不到关联帧直接返回
		if (!frame->getAllRelatedFrames(relatedFrameID)) return false;

		//如果全局姿态已知，这里就不求了
		if (frame->isGlobalPoseKnown())
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
				只有关联帧没有其他关联帧或者没有被其他帧关联的情况下才可以使用对极几何来初始化，
				关联帧如果有其他关联帧，说明该关联帧肯定存在路点，那么肯定属于某一个地图中，
				所以如果这里又用对极几何就会产生一个新的地图，将产生矛盾。
				该函数只考虑计算当前帧的位姿的情况，多个地图的拼接问题由其他函数考虑。
				*/
				if (frame->getRelatedFrame(i)->getRelatedFrame()->hasRelatedFrame()!=1)
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
			auto posetmp = std::get<Sophus::SE3d>(Frameid2ErrNPose[LeastErrorID]) 
				* frame->getRelatedFrame(LeastErrorID)->getRelatedFrame()->getGlobalPose();
			frame->setGlobalPose(posetmp);

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
	//assert(LargestValue !=0);

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

	auto K = std::dynamic_pointer_cast<PinholeFrameObject>(current)->CameraMatrix;
	current->setGlobalPose(this->SolvePNP(cvMapPoint, cvKeyPoint, K));
	//int MapID = GlobalMap->getFrameObjectMapID(MappointNumber[LargestIndex].front()->getID());
	//GlobalMap->updateFrameObject(current, MapID);
	current->MapID = LargestIndex;
	return true;
}

bool PinholePoseReconstructor::SolveWithMappoint(FrameObject::Ptr current, FrameObject::RelatedFrameInfo::Ptr related, GlobalMapObject::Ptr GlobalMap)
{
	auto referencePtr = related->getRelatedFrame();
	std::vector<cv::Point3d> cvMapPoint;
	std::vector<cv::Point2d> cvKeyPoint;
	for (auto& item : related->KeyPointMatch)
	{
		if (current->hasMappoint(item.queryIdx))
		{
			auto MapPoint = current->getMapPoint(item.queryIdx)->Position;
			auto KeyPoint = referencePtr->KeyPoints.at(item.trainIdx).pt;
			cvMapPoint.emplace_back(MapPoint(0), MapPoint(1), MapPoint(2));
			cvKeyPoint.emplace_back(KeyPoint);
		}
	}
	
	if (cvMapPoint.size() < 4) //PNP最低点数要求
		return false;

	auto K = std::dynamic_pointer_cast<PinholeFrameObject>(referencePtr)->CameraMatrix;
	auto pose = this->SolvePNP(cvMapPoint, cvKeyPoint, K);
	referencePtr->setGlobalPose(pose);
	referencePtr->MapID = current->MapID;

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

	cv::Mat1d E_Mat, E_R, E_t, E_T;
	cv::recoverPose(SrcPoints, DstPoints, PinholeCurrent->CameraMatrix, cv::noArray(),
		ReferencedFrame->CameraMatrix, cv::noArray(), E_Mat, E_R, E_t, cv::RANSAC);
	E_T = DataFlowObject::Rt2T(E_R, E_t);
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
	K1_invtmp.copyTo(K1_inv.colRange(0, 3).rowRange(0, 3));
	
	cv::Mat1d K2 = cv::Mat1d::zeros(3, 4);
	CameraMat2.copyTo(K2.colRange(0, 3).rowRange(0, 3));
	
	cv::Mat H_Mat = K2 * T.inv() * K1_inv;
	
	double Error = 0;
	for (size_t i = 0; i < src.size(); i++)
	{
		cv::Vec3d p1(src[i].x, src[i].y, 1);
		cv::Vec3d p2(dst[1].x, dst[i].y, 1);
		
		cv::Mat1d project = H_Mat * p1;
		
		double localerror = (p2(0) - project(0)) * (p2(0) - project(0)) +
			(p2(1) - project(1)) * (p2(1) - project(1));
		
		Error += localerror;
	}
	Error /= src.size();
	return Error;
}

void PinholePoseReconstructor::ComputeMappoint(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
	//考虑计算新路点的问题和地图融合的问题
	//计算路点思路：
	/*
	计算所有与已知全局坐标的关联帧间相对姿态(同一地图中)
	根据相对姿态和特征点匹配信息计算新路点
	
	找出所有姿态未知的关联帧合集(单帧地图)
	while 合集中帧数量减少
		根据路点和匹配特征点计算姿态
		计算新路点
	end while
	//TODO: 地图融合问题？ sim(3)?
	*/

	std::set<FrameObject::RelatedFrameInfo::Ptr> SameMapWithPose;
	std::set<FrameObject::RelatedFrameInfo::Ptr> SameMapWithoutPose;
	std::set<FrameObject::RelatedFrameInfo::Ptr> DifferentMap;

	int CurrentMapID = frame->MapID;
	std::set<int> RelatedFrameID;
	frame->getAllRelatedFrames(RelatedFrameID);
	for (auto& ID : RelatedFrameID)
	{
		auto RelatedFrameInfoPtr = frame->getRelatedFrame(ID);
		auto RelatedFramePtr = RelatedFrameInfoPtr->getRelatedFrame();
		if (RelatedFramePtr->MapID == CurrentMapID)
		{
			if (RelatedFramePtr->isGlobalPoseKnown())
			{
				SameMapWithPose.insert(RelatedFrameInfoPtr);
			}
			else
			{
				std::cout << this->type_name() << ": Error in same Map but known Pose, frame ID = "<<RelatedFramePtr->getID() << std::endl;
				SameMapWithoutPose.insert(RelatedFrameInfoPtr);
			}
		}
		else
		{
			//如果该关联帧没有关联帧，说明是单帧地图，可以直接先按照本地图的帧处理，计算出位姿后即可加入本地图，否则不能加入
			if (RelatedFramePtr->hasRelatedFrame()==1)
			{
				SameMapWithoutPose.insert(RelatedFrameInfoPtr);
			}
			else
			{
				DifferentMap.insert(RelatedFrameInfoPtr);
			}
		}
	}

	int unknownPoseCount = SameMapWithoutPose.size();
	while (true)
	{
		//计算新路点
		this->GenNewMappoints(frame, SameMapWithPose, GlobalMap);

		//根据新增路点计算其他帧姿态
		for (auto iter = SameMapWithoutPose.begin(); iter != SameMapWithoutPose.end();)
		{
			//如果成功计算姿态，那么就加入到本地图中
			if (this->SolveWithMappoint(frame, *iter, GlobalMap))
			{
				(*iter)->getRelatedFrame()->MapID = frame->MapID;
				SameMapWithPose.insert(*iter);
				iter = SameMapWithoutPose.erase(iter);
			}
			else
			{
				iter++;
			}
		}


		//当不知道的姿态的帧为0或者不再减少时证明已经无法获得帧姿态信息了，退出
		if (unknownPoseCount == 0)
			break;
		if (unknownPoseCount == SameMapWithoutPose.size())
			break;

		unknownPoseCount = SameMapWithoutPose.size();
	}
}

void PinholePoseReconstructor::GenNewMappoints(FrameObject::Ptr frame, std::set<FrameObject::RelatedFrameInfo::Ptr>& Related, GlobalMapObject::Ptr GlobalMap)
{
	auto pinholeframe1 = std::dynamic_pointer_cast<PinholeFrameObject>(frame);
	cv::Mat1d Pose1;
	DataFlowObject::Sophus2cvMat(pinholeframe1->getGlobalPose().inverse(), Pose1);
	cv::Mat1d P1 = DataFlowObject::CameraMatrix3x4(pinholeframe1->CameraMatrix) * Pose1; //计算投影矩阵

	//<关联帧信息，投影矩阵>
	std::map<FrameObject::RelatedFrameInfo::Ptr, cv::Mat1d> FramePMap; //投影矩阵列表
	
	//<本帧特征点编号，list<关联帧信息,对应特征点编号>>
	std::map<int, std::list<std::tuple<FrameObject::RelatedFrameInfo::Ptr, int>>> MatchedPointsList; //未知路点列表
	for (auto& item : Related)
	{
		auto PinholePointer = std::dynamic_pointer_cast<PinholeFrameObject>(item->getRelatedFrame());
		cv::Mat1d Pose;
		DataFlowObject::Sophus2cvMat(PinholePointer->getGlobalPose().inverse()  , Pose);
		cv::Mat1d P = DataFlowObject::CameraMatrix3x4(PinholePointer->CameraMatrix) * Pose;
		FramePMap.insert({ item,P });

		//std::set<int> MappointKeyID;
		//PinholePointer->getAllMappointID(MappointKeyID);
		
		for (auto& match : item->KeyPointMatch)
		{
			if (MatchedPointsList.count(match.queryIdx) == 0)
			{
				MatchedPointsList.insert({ match.queryIdx,std::list<std::tuple<FrameObject::RelatedFrameInfo::Ptr, int>>() });
			}
			MatchedPointsList.at(match.queryIdx).push_back({ item,match.trainIdx });
		}
	}
	
	//删除已有路点的点
	for (auto iter = MatchedPointsList.begin(); iter != MatchedPointsList.end();)
	{
		bool hasRoadPoint = false;
		MapPointObject::Ptr EmptyPoint;
		for (auto&[ptr,pointid] : iter->second)
		{
			if (ptr->getRelatedFrame()->hasMappoint(pointid))
			{
				hasRoadPoint = true;
				EmptyPoint = ptr->getRelatedFrame()->getMapPoint(pointid);
			}
		}
		if (hasRoadPoint)
		{
			pinholeframe1->addMapPoint(iter->first, EmptyPoint, pinholeframe1->getGlobalPose().inverse() * EmptyPoint->Position);
			for (auto& [ptr,pointid] : iter->second)
			{
				ptr->getRelatedFrame()->addMapPoint(pointid, EmptyPoint,
					ptr->getRelatedFrame()->getGlobalPose().inverse() * EmptyPoint->Position);
			}
			iter = MatchedPointsList.erase(iter);
		}
		else
		{
			iter++;
		}
	}

	//使用多张图来更精确的计算路点
	for (auto& [key,value] : MatchedPointsList)
	{
		cv::Mat1d X;
		cv::Mat1d A = cv::Mat1d::zeros(value.size() * 2 + 2, 4);
		cv::Mat row1, row2;
		auto Point1 = pinholeframe1->KeyPoints.at(key).pt;
		row1 = Point1.x * P1.row(2) - P1.row(0);
		row2 = Point1.y * P1.row(2) - P1.row(1);
		row1.copyTo(A.row(0));
		row2.copyTo(A.row(1));

		int count = 2;

		cv::Mat1d P2_;
		cv::Point2f p2testreal;

		for (auto& item : value)
		{
			auto relatedptr = std::get<0>(item);
			auto point = std::get<1>(item);
			auto ptr = relatedptr->getRelatedFrame();

			auto Point2 = ptr->KeyPoints.at(point).pt;
			auto P2 = FramePMap.at(relatedptr);
			row1 = Point2.x * P2.row(2) - P2.row(0);
			row2 = Point2.y * P2.row(2) - P2.row(1);

			row1.copyTo(A.row(count++));
			row2.copyTo(A.row(count++));

			P2_ = P2;
			p2testreal = Point2;
		}

		cv::Mat1d Atest(A.size());
		A.copyTo(Atest);
		A.row(3).copyTo(Atest.row(1));
		A.row(1).copyTo(Atest.row(3));

		cv::SVD::solveZ(Atest, X);
		//TODO: 解决值为负数的情况
		X = X / X(3);//TODO: 寻找一下更好的归一化方法

		/** *******/
		cv::Mat1d p1test = P1 * X;
		p1test = p1test / p1test(2);
		cv::Mat1d p2test = P2_ * X;
		p2test = p2test / p2test(2);
		/** **************/

		Eigen::Vector4d EigenPoint;
		cv::cv2eigen(X, EigenPoint);
		//终于产生一个路点了！！！！不容易啊！
		auto MappointPtr = MapPointObject::Create(GlobalMap->AssignMappointID());
		MappointPtr->Position = EigenPoint;
		
		//向路点添加观测的帧
		MappointPtr->addObservation(pinholeframe1, key);
		for (auto& item : value)
		{
			auto relatedptr = std::get<0>(item);
			auto point = std::get<1>(item);
			auto ptr = relatedptr->getRelatedFrame();
			MappointPtr->addObservation(ptr, point);
		}
		
		GlobalMap->addMapPoint(MappointPtr); //立即向全局地图中加入路点，不然指针就无了

		pinholeframe1->addMapPoint(key, MappointPtr, pinholeframe1->getGlobalPose().inverse() * EigenPoint);
		
		for (auto& item : value)
		{
			auto ptr = std::get<0>(item)->getRelatedFrame();
			auto pointid = std::get<1>(item);
			ptr->addMapPoint(pointid, MappointPtr, ptr->getGlobalPose().inverse() * EigenPoint);
		}
	}
}

Sophus::SE3d PinholePoseReconstructor::SolvePNP(std::vector<cv::Point3d>& WorldPoints, std::vector<cv::Point2d>& CameraPoints, cv::Mat1d& Intrinsic)
{
	cv::Mat1d rvec, R, t, T;
	cv::solvePnP(WorldPoints, CameraPoints, Intrinsic, cv::noArray(), R, t);
	R = cv::Mat1d::zeros(3, 3);
	cv::Rodrigues(rvec, R);
	T = DataFlowObject::Rt2T(R, t);
	Sophus::SE3d pose;
	DataFlowObject::cvMat2Sophus(T, pose);
	return pose;
}
