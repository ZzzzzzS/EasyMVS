#include "EasyMVSWarpper.h"
#include "common/JsonSaver.hpp"

EasyMVSWarpper::EasyMVSWarpper()
{
    qRegisterMetaType<std::map<int, Sophus::SE3d>>("ReturnType");
    //Create Object
    this->GlobalMap = GlobalMapObject::Create();
    this->CameraLeft = PinholeMemImgReader::Create();
    this->CameraRight = PinholeMemImgReader::Create();
    this->Photographer1 = PinholePhotographer::Create(this->GlobalMap, { CameraRight, CameraLeft });
    this->Extractor1 = ReflectiveStickerExtractor::Create();
    this->Matcher1 = ReflectiveStickerMatcher::Create(GlobalMap);
    this->PoseReconstructor1 = ReflectiveReconstructor::Create(GlobalMap);

    QObject::connect(this, &EasyMVSWarpper::TriggerOnce, Photographer1.get(), 
        static_cast<void (WorkFlowObject::*)(void)>(&PinholePhotographer::Trigger),Qt::QueuedConnection); //使用队列连接，防止栈深度太深
	
    QObject::connect(Photographer1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        Extractor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&CVFeatureExtractor::Trigger));

    QObject::connect(Extractor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        Matcher1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&VocTreeMatcher::Trigger));

    QObject::connect(Matcher1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        PoseReconstructor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&PoseReconstructor::Trigger));

    QObject::connect(PoseReconstructor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        this, &EasyMVSWarpper::FinishedOnce);
}

EasyMVSWarpper::EasyMVSWarpper(cv::InputArray LeftCameraMatrix, cv::InputArray LeftDistcoeff, cv::InputArray RightCameraMatrix, cv::InputArray RightDistcoeff, cv::InputArray TL2R)
{
    qRegisterMetaType<std::map<int, Sophus::SE3d>>("ReturnType");
    //Create Object
    this->GlobalMap = GlobalMapObject::Create();
    this->CameraLeft = PinholeMemImgReader::Create(LeftCameraMatrix.getMat(), LeftDistcoeff.getMat());
    this->CameraRight = PinholeMemImgReader::Create(RightCameraMatrix.getMat(),RightDistcoeff.getMat());

    Sophus::SE3d ExtrinsicRight(Sophus::Matrix4d::Identity());
    Sophus::SE3d ExtrinsicLeft;
	FrameObject::cvMat2Sophus(TL2R.getMat(), ExtrinsicLeft);
    this->Photographer1 = PinholePhotographer::Create(this->GlobalMap, { {CameraRight,ExtrinsicRight}, {CameraLeft,ExtrinsicLeft} });
    this->Extractor1 = ReflectiveStickerExtractor::Create();
    this->Matcher1 = ReflectiveStickerMatcher::Create(GlobalMap);
    this->PoseReconstructor1 = ReflectiveReconstructor::Create(GlobalMap);

    QObject::connect(this, &EasyMVSWarpper::TriggerOnce, Photographer1.get(),
        static_cast<void (WorkFlowObject::*)(void)>(&PinholePhotographer::Trigger), Qt::QueuedConnection); //使用队列连接，防止栈深度太深

    QObject::connect(Photographer1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        Extractor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&CVFeatureExtractor::Trigger));

    QObject::connect(Extractor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        Matcher1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&VocTreeMatcher::Trigger));

    QObject::connect(Matcher1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        PoseReconstructor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&PoseReconstructor::Trigger));

    QObject::connect(PoseReconstructor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        this, &EasyMVSWarpper::FinishedOnce,Qt::QueuedConnection);
	
	
}

EasyMVSWarpper::~EasyMVSWarpper()
{

}

bool EasyMVSWarpper::LoadParameters(std::string filename)
{
    try
    {
        std::ifstream settings(filename);
        JsonNode Json = JsonNode::parse(settings);
        settings.close();

        //config Object
        //不从配置文件加载内外参数，使用指定的内外参
        /*if (JsonNode CameraNode = Json["CameraRight"]; !CameraRight->load(CameraNode))
        {
            std::cerr << "failed to load camera settings" << std::endl;
        }

        if (JsonNode CameraNode = Json["CameraLeft"]; !CameraLeft->load(CameraNode))
        {
            std::cerr << "failed to load camera settings" << std::endl;
        }

        if (JsonNode PhotographerNode = Json["Photographer1"]; !Photographer1->load(PhotographerNode))
        {
            std::cerr << "failed to load photographer settings" << std::endl;
        }*/

        if (!CameraLeft->open())
        {
            std::cerr << "failed to open camera" << std::endl;
        }

        if (!CameraRight->open())
        {
            std::cerr << "failed to open camera" << std::endl;
        }

        if (auto ExtractorNode = Json["FeatureExtractor1"]; !Extractor1->load(ExtractorNode))
        {
            std::cerr << "failed to load extractor" << std::endl;
        }

        if (auto MatcherNode = Json["FeatureMatcher1"]; !Matcher1->load(MatcherNode))
        {
            std::cerr << "failed to load matcher" << std::endl;
        }

        if (auto PoseReconstructorNode = Json["PoseReconstructor1"]; !PoseReconstructor1->load(PoseReconstructorNode))
        {
            std::cerr << "failed to load PoseReconstructor" << std::endl;
        }

        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}

bool EasyMVSWarpper::SaveParameters(std::string filename)
{
    try
    {
        MVSConfig::GlobalConfig::WorkspacePath = filename;
        JsonNode Json;
        CameraLeft->save(Json["CameraLeft"]);
        CameraRight->save(Json["CameraRight"]);
        Photographer1->save(Json["Photographer1"]);
        Extractor1->save(Json["FeatureExtractor1"]);
        Matcher1->save(Json["FeatureMatcher1"]);
        PoseReconstructor1->save(Json["PoseReconstructor1"]);
        std::ofstream settings(filename);
        settings << std::setw(4) << Json;
        settings.close();
        return true;
    }
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
    return false;
}

bool EasyMVSWarpper::SaveData(std::string filename)
{
    std::set<int> FrameID;
    this->GlobalMap->getAllFrameObjectID(FrameID);
    if (FrameID.empty())
    {
        std::cout << "EasyMVSWarpper: failed to save, current Map has no frame" << std::endl;
        return false;
    }

    auto path = QString::fromStdString(filename);
    if (path.isEmpty())
    {
		std::cout << "EasyMVSWarpper: failed to save, path is empty" << std::endl;
        return false;
    }

    try
    {
        std::ofstream out(path.toStdString());
        JsonNode root;
        for (auto& item : FrameID)
        {
            auto FramePtr = this->GlobalMap->getFrameObject(item);
            JsonNode FrameNode;
            FrameNode["ID"] = item;
            FrameNode["Pose"] = FramePtr->getGlobalPose();
            root.push_back(FrameNode);
        }

        out << std::setw(4) << root;
        out.close();
        return true;
    }
    catch (const std::exception& e)
    {
        std::cout << "EasyMVSWarpper: Failed to save" << e.what() << std::endl;
    }
    return false;
}

bool EasyMVSWarpper::LoadImages(cv::InputArrayOfArrays Left, cv::InputArrayOfArrays Right)
{
    if (this->FrameCount != 0)
    {
		std::cout << "EasyMVSWarpper: failed to load images, current rebuild process is not finished" << std::endl;
		return false;
    }
    std::vector<cv::Mat> LeftArray, RightArray;
    Left.getMatVector(LeftArray);
    Right.getMatVector(RightArray);
    if ((LeftArray.size() != RightArray.size()) || (LeftArray.empty()))
    {
		std::cout << "EasyMVSWarpper: failed to load images, input images are not valid " << std::endl;
        return false;
    }

    auto MatcherPtr = std::dynamic_pointer_cast<ReflectiveStickerMatcher>(this->Matcher1);
    MatcherPtr->CurrentFrameIndex = 0;
    MatcherPtr->TotalFrameNumber = LeftArray.size();
    MatcherPtr->LastFrame = nullptr;
    MatcherPtr->FirstFrame = nullptr;
	
    std::dynamic_pointer_cast<PinholeMemImgReader>(this->CameraLeft)->release();
    std::dynamic_pointer_cast<PinholeMemImgReader>(this->CameraRight)->release();
    std::dynamic_pointer_cast<PinholeMemImgReader>(this->CameraLeft)->addImages(Left);
    std::dynamic_pointer_cast<PinholeMemImgReader>(this->CameraRight)->addImages(Right);
    return true;
}

void EasyMVSWarpper::Trigger()
{
    if (this->FrameCount != 0)
    {
		std::cout << "EasyMVSWarpper: failed to trigger, current rebuild process is not finished " << std::endl;
		return;
    }
    this->FrameCount = std::dynamic_pointer_cast<PinholeMemImgReader>(this->CameraLeft)->getBufferSize();
    this->GlobalMap->removeAllFrameObject();
    this->GlobalMap->removeAllMapPoint();
	
    emit this->TriggerOnce();
}

void EasyMVSWarpper::FinishedOnce(WorkFlowObject::DataQueue queue)
{
    this->FrameCount--;
    if (this->FrameCount != 0)
    {
        emit this->TriggerOnce();
    }
    else
    {
        std::set<int>FrameID;
        this->GlobalMap->getAllFrameObjectID(FrameID);
        std::map<int, Sophus::SE3d> Poses;
        for (auto& item : FrameID)
        {
            auto FramePtr = this->GlobalMap->getFrameObject(item);
            auto pose = FramePtr->getGlobalPose();
            if (FramePtr->MapID == 0 && FramePtr->isGlobalPoseKnown())
            {
                Poses.insert({ item,pose });
            }
        }
		emit this->Finished(Poses);
    }
}
