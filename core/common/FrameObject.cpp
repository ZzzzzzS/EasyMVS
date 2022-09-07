﻿#include "FrameObject.h"
#include "MVSConfig.h"
#include "JsonSaver.hpp"
#include "MapPointObject.h"

FrameObject::Ptr FrameObject::Create(int ID, uint32_t Timestamp)
{
    return std::make_shared<FrameObject>(ID, Timestamp);
}

FrameObject::Ptr FrameObject::Create(int ID, const cv::Mat& RGBMat, uint32_t Timestamp, const cv::Mat& XYZMat)
{
    auto ptr = std::make_shared<FrameObject>(ID, Timestamp);
    ptr->RGBMat = RGBMat;
    ptr->XYZMat = XYZMat;
    return ptr;
}

FrameObject::FrameObject(int ID, uint32_t Timestamp)
    :FrameID(ID),
    Timestamp(Timestamp)
{
}

FrameObject::FrameObject()
    :FrameID(-1),
    Timestamp(-1)
{

}

FrameObject::~FrameObject()
{
}

int FrameObject::getID()
{
    return this->FrameID;
}

bool FrameObject::setTimestamp(uint32_t time)
{
    if (time < 0)
    {
        return false;
    }
    this->Timestamp = time;
    return true;
}

uint32_t FrameObject::getTimestamp()
{
    return this->Timestamp;
}

bool FrameObject::setBestFrame(RelatedFrameInfo::Ptr FramePtr)
{
    try
    {
        if (auto id = FramePtr->getRelatedFrame()->getID(); this->RelatedFrame.count(id) == 0)
        {
            std::cerr << "best frame is not within related frames\n";
            return false;
        }
        this->BestCamera = FramePtr;
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}

FrameObject::RelatedFrameInfo::Ptr FrameObject::getBestFrame()
{
    if (this->BestCamera == nullptr)
    {
        std::cerr << "Best camera is empty!" << std::endl;
    }
    return this->BestCamera;
}

bool FrameObject::addRelatedFrame(RelatedFrameInfo::Ptr FramePtr)
{
    try
    {
        if (auto id = FramePtr->getRelatedFrame()->getID(); this->RelatedFrame.count(id) != 0)
        {
            this->RelatedFrame[id] = FramePtr;
            return true;
        }
        else
        {
            std::cerr << "frame already exist" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}


bool FrameObject::removeRelatedFrame(int FrameID)
{
    if (this->RelatedFrame.count(FrameID) == 0)
    {
        std::cerr << "frame dose NOT exist, ID=" << FrameID << std::endl;
        return false;
    }
    else
    {
        this->RelatedFrame.erase(FrameID);
        return true;
    }
}

bool FrameObject::removeAllRelatedFrames()
{
    this->RelatedFrame.clear();
    return true;
}

bool FrameObject::updateRelatedFrame(RelatedFrameInfo::Ptr FramePtr)
{
    try
    {
        if (auto id = FramePtr->getRelatedFrame()->getID(); this->RelatedFrame.count(id) != 0)
        {
            this->RelatedFrame[id] = FramePtr;
        }
        else
        {
            std::cerr << "the frame dose NOT exist" << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}

FrameObject::RelatedFrameInfo::Ptr FrameObject::getRelatedFrame(int FrameID)
{
    try
    {
        auto ptr = this->RelatedFrame.at(FrameID);
        return  ptr;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return RelatedFrameInfo::Ptr();
}

bool FrameObject::getAllRelatedFrames(std::set<RelatedFrameInfo::Ptr>& Frames)
{
    if (this->RelatedFrame.empty())
        return false;
    
    for (auto&& item : this->RelatedFrame)
    {
        Frames.insert(item.second);
    }
    return true;

}

bool FrameObject::getAllRelatedFrames(std::set<int>& FrameID)
{
    if (this->RelatedFrame.empty())
        return false;

    for (auto&& item: this->RelatedFrame)
    {
        FrameID.insert(item.first);
    }
    return true;
}

bool FrameObject::addMapPoint(int KeyPointID, std::shared_ptr<MapPointObject> MapPoint, const Eigen::Vector4d& LocalCoordinate)
{
    try
    {
        if (this->ObservedMapPoints.count(KeyPointID) != 0)
        {
            std::cerr << "Mappoint already exist, keypoint id=" << KeyPointID << std::endl;
            return false;
        }

        this->ObservedMapPoints[KeyPointID] = { MapPoint,LocalCoordinate,MapPoint->getID() };
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}

bool FrameObject::removeMapPoint(int KeyPointID)
{
    try
    {
        if (this->ObservedMapPoints.erase(KeyPointID) == 0)
        {
            std::cerr << "the mappoint dose NOT exist, keypoint id=" << KeyPointID << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}

bool FrameObject::removeAllMapPoints()
{
    if(this->ObservedMapPoints.empty())
        return false;
    else
    {
        this->ObservedMapPoints.clear();
        return true;
    }
}

bool FrameObject::getMapPoint(int KeyPointID, std::shared_ptr<MapPointObject>& MapPoint, Eigen::Vector4d& LocalCoordinate)
{
    try
    {
        MapPoint = std::get<0>(this->ObservedMapPoints[KeyPointID]).lock();
        LocalCoordinate = std::get<1>(this->ObservedMapPoints[KeyPointID]);
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}

bool FrameObject::getMapPoint(int KeyPointID, int& MappointID, Eigen::Vector4d& LocalCoordinate)
{
    try
    {
        MappointID = std::get<2>(this->ObservedMapPoints[KeyPointID]);
        LocalCoordinate = std::get<1>(this->ObservedMapPoints[KeyPointID]);
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}

bool FrameObject::updateMapPoint(int KeyPointID, std::shared_ptr<MapPointObject> MapPoint, const Eigen::Vector4d& LocalCoordinate)
{
    try
    {
        this->ObservedMapPoints.at(KeyPointID) = std::make_tuple(MapPoint, LocalCoordinate, MapPoint->getID());
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}

bool FrameObject::load(JsonNode& fs)
{
    try
    {
        //read coefficents
        this->FrameID = fs.at("id").get<int>();
        this->Timestamp = fs.at("timestamp").get<int>();
        this->KeyPoints = fs.at("key-points");
        this->KeyPointsDescriptors = fs.at("key-points-descriptors");
        this->GlobalPose = fs.at("global-pose");

        //read rgb mat
        if (!fs.at("rgb-mat").is_null())
        {
            std::string path = fs.at("rgb-mat");
            this->RGBMat = cv::imread(path);
            if (this->RGBMat.empty())
            {
                throw std::exception("can NOT read rgb mat");
            }
        }
        else
        {
            this->RGBMat = cv::Mat();
        }

        //read xyz mat
        if (!fs.at("xyz-mat").is_null())
        {
            std::string path = fs.at("xyz-mat");
            this->XYZMat = cv::imread(path);
            if (this->XYZMat.empty())
            {
                throw std::exception("can NOT read XYZ mat");
            }
        }
        else
        {
            this->XYZMat = cv::Mat();
        }

        //read mappoint without pointer
        JsonNode mappointnode = fs.at("observed-mappoint");
        for(auto & item : mappointnode)
        {
            int id = item.at("keypoint-id");
            JsonNode pos = item.at("local-position");
            Eigen::Vector4d tmppos(pos.at(0), pos.at(1), pos.at(2), pos.at(3));

            ObservedMapPoints[id] = { MapPointObject::Ptr(),tmppos,item.at("mappoint-id")}; //c++17的语法真高级
        }

        //read related frame without pointer
        JsonNode relatedframenode = fs.at("related-frame");
        for (auto& item : relatedframenode)
        {
            auto tmpframe = std::make_shared<RelatedFrameInfo>();
            if (!tmpframe->load(item))
            {
                throw std::exception("failed to load related frame");
            }

            this->RelatedFrame[item.at("frame-id")] = tmpframe;
        }

        //read best frame
        if (!fs.at("best-camera-id").is_null())
        {
            int bestnode = fs.at("best-camera-id");
            this->BestCamera = this->RelatedFrame.at(bestnode);
        }

        return true;
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
    return false;
}

bool FrameObject::save(JsonNode& fs)
{
    // create path to save
    std::string path = MVSConfig::GlobalConfig::WorkspacePath;
    path += std::to_string(this->FrameID) + "/";
    if (!this->mkdir(path))
    {
        return false;
    }

    try
    {
        //save parameters
        fs = {
            {"type-id",this->type_name()},
            {"id",this->FrameID},
            {"timestamp",this->Timestamp}
        };

        if (this->BestCamera != nullptr)
        {
            fs["best-camera-id"] = this->BestCamera->getRelatedFrame()->getID();
        }
        else
        {
            fs["best-camera-id"] = JsonNode::object();
        }

        fs["global-pose"] = this->GlobalPose;
        fs["key-points"] = this->KeyPoints;
        fs["key-points-descriptors"] = this->KeyPointsDescriptors;
        

        //save related frame
        JsonNode RelatedFrameNode = JsonNode::array();
        for (auto&& item : this->RelatedFrame)
        {
            JsonNode temp;
            if (!item.second->save(temp))
            {
                throw std::exception("failed to save related frame");
            }
            RelatedFrameNode.push_back(temp);
        }
        fs["related-frame"] = RelatedFrameNode;

        //save mappoint
        JsonNode ObservedMapPointNode = JsonNode::array();
        for (auto&& item : this->ObservedMapPoints)
        {
            JsonNode temp;
            temp["keypoint-id"] = item.first;
            
            //syntax from c++ 17
            if (auto MappointTmp = std::get<0>(item.second).lock(); MappointTmp != nullptr)
            {
                temp["mappoint-id"] = MappointTmp->getID();
            }
            else
            {
                throw std::exception("the mappoint related to keypoint does NOT exist");
            }

            //temp["mappoint-id"]
            temp["local-position"] = {
                std::get<1>(item.second)(0),
                std::get<1>(item.second)(1),
                std::get<1>(item.second)(2),
                std::get<1>(item.second)(3)
            };
            ObservedMapPointNode.push_back(temp);
        }
        fs["observed-mappoint"] = ObservedMapPointNode;

        //save rgb mat
        if (!RGBMat.empty())
        {
            std::string rgbpath = path + "rgb-mat" + MVSConfig::GlobalConfig::RGBMatType;
            fs["rgb-mat"] = rgbpath;
            if (!cv::imwrite(rgbpath, this->RGBMat))
            {
                throw std::exception("failed to save rgb mat");
            }
        }
        else
        {
            fs["rgb-mat"] = JsonNode::object();
        }

        //save xyz mat
        if (!XYZMat.empty())
        {
            auto xyzpath = path + "xyz-mat" + MVSConfig::GlobalConfig::XYZMatType;
            fs["xyz-mat"] = xyzpath;
            if (!cv::imwrite(xyzpath, this->XYZMat))
            {
                throw std::exception("failed to save xyz mat");
            }
        }
        else
        {
            fs["xyz-mat"] = JsonNode::object();
        }
        return true;
    }
    catch (const std::exception& e)
    {
        fs = JsonNode::object();
        std::cout << e.what() << std::endl;
    }
    
    return false;
}

std::string FrameObject::type_name()
{
    return std::string("frame-object");
}

PinholeFrameObject::Ptr PinholeFrameObject::Create(int ID, uint32_t Timestamp)
{
    return std::make_shared<PinholeFrameObject>(ID, Timestamp);
}

PinholeFrameObject::Ptr PinholeFrameObject::Create(int ID, const cv::Mat& CameraMatrix, const cv::Mat& DistCoeff, uint32_t Timestamp, const cv::Mat& RGBMat, const cv::Mat& XYZMat)
{
    auto ptr = std::make_shared<PinholeFrameObject>(ID, Timestamp);
    ptr->CameraMatrix = CameraMatrix;
    ptr->DistCoeff = DistCoeff;
    ptr->RGBMat = RGBMat;
    ptr->XYZMat = XYZMat;
    return ptr;
}

PinholeFrameObject::PinholeFrameObject(int ID, uint32_t Timestamp)
    :FrameObject(ID,Timestamp)
{
}

PinholeFrameObject::~PinholeFrameObject()
{
}

bool PinholeFrameObject::load(JsonNode& fs)
{
    //call parent load method first
    if (!FrameObject::load(fs))
    {
        return false;
    }

    try
    {
        this->CameraMatrix = fs.at("intrinsic-matrix").get<cv::Mat>();
        this->DistCoeff = fs.at("distcoeff-matrix").get<cv::Mat>();

        return true;
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
    return false;
}

bool PinholeFrameObject::save(JsonNode& fs)
{
    //call parent save method
    if (!FrameObject::save(fs))
    {
        return false;
    }

    try
    {
        fs["type-id"] = this->type_name();
        fs["intrinsic-matrix"] = this->CameraMatrix;
        fs["distcoeff-matrix"] = this->DistCoeff;
        return true;
    }
    catch (const std::exception& e)
    {
        fs = JsonNode::object();
        std::cout << e.what() << std::endl;
    }
    return false;
}

std::string PinholeFrameObject::type_name()
{
    return std::string("pinhole-frame-object");
}

FrameObject::RelatedFrameInfo::Ptr FrameObject::RelatedFrameInfo::Create(std::shared_ptr<FrameObject> RelatedFrame)
{
    return std::make_shared<RelatedFrameInfo>(RelatedFrame);
}

FrameObject::RelatedFrameInfo::Ptr FrameObject::RelatedFrameInfo::Create(std::shared_ptr<FrameObject> RelatedFrame, std::shared_ptr<Sophus::SE3d> Pose, double sigma)
{
    auto ptr = Create(RelatedFrame);
    ptr->Pose = Pose;
    ptr->sigma = sigma;
    return ptr;
}

FrameObject::RelatedFrameInfo::Ptr FrameObject::RelatedFrameInfo::Create(std::shared_ptr<FrameObject> RelatedFrame,const std::vector<cv::DMatch>& KeyPointMatch)
{
    auto ptr = Create(RelatedFrame);
    ptr->KeyPointMatch = KeyPointMatch;
    return ptr;
}

std::shared_ptr<FrameObject> FrameObject::RelatedFrameInfo::getRelatedFrame()
{
    auto ptr = this->RelatedFramePtr.lock();
    if (ptr == nullptr)
    {
        std::cerr << "the related frame does NOT exist" << std::endl;
    }
    return ptr;
}


bool FrameObject::RelatedFrameInfo::setRelatedFrame(FrameObject::Ptr frame)
{
    if (frame == nullptr)
        return false;
    this->RelatedFramePtr = frame;
    return true;
}


bool FrameObject::RelatedFrameInfo::isFrameExist()
{
    return (this->RelatedFramePtr.lock() != nullptr) ? true : false;
}

bool FrameObject::RelatedFrameInfo::save(JsonNode& fs)
{
    try
    {
        fs["sigma"] = this->sigma;
        fs["keypoint-match"] = this->KeyPointMatch;
        fs["extrinsic-matrix"] = *this->Pose;

        auto FramePtr = this->RelatedFramePtr.lock();
        if (FramePtr == nullptr)
        {
            fs = JsonNode::object();
            throw std::exception("related frame dose NOT exist");
            return false;
        }
        fs["frame-id"] = FramePtr->FrameID;
        return true;
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
    return false;
}

bool FrameObject::RelatedFrameInfo::load(JsonNode& fs)
{
    try
    {
        this->sigma = fs.at("sigma").get<double>();
        this->KeyPointMatch = fs.at("keypoint-match");
        Sophus::SE3d tmppose = fs.at("extrinsic-matrix").get<Sophus::SE3d>();
        this->Pose = std::make_shared<Sophus::SE3d>(std::move(tmppose));
        //this->RelatedFrameID = fs["id"].get<int>();
        return true;
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
    return false;
}

std::string FrameObject::RelatedFrameInfo::type_name()
{
    return std::string("related-frame-info");
}

FrameObject::RelatedFrameInfo::RelatedFrameInfo()
    :sigma(0)
{

}

FrameObject::RelatedFrameInfo::RelatedFrameInfo(std::shared_ptr<FrameObject> RelatedFrame)
    :sigma(0),
    RelatedFramePtr(RelatedFrame)
{
}

FrameObject::RelatedFrameInfo::~RelatedFrameInfo()
{
}
