#include "FrameObject.h"
#include "MVSConfig.h"
#include "JsonSaver.hpp"

FrameObject::Ptr FrameObject::Create(int ID, uint32_t Timestamp)
{
    return Ptr();
}

FrameObject::Ptr FrameObject::Create(int ID, const cv::Mat& RGBMat, uint32_t Timestamp, const cv::Mat& XYZMap)
{
    return Ptr();
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

int FrameObject::getFrameID()
{
    return this->FrameID;
}

uint32_t FrameObject::getTimestamp()
{
    return this->Timestamp;
}

bool FrameObject::setBestFrame(RelatedFrameInfo::Ptr FramePtr)
{
    return false;
}

FrameObject::RelatedFrameInfo::Ptr FrameObject::getBestFrame()
{
    return RelatedFrameInfo::Ptr();
}

bool FrameObject::addRelatedFrame(RelatedFrameInfo::Ptr FramePtr)
{
    return false;
}


bool FrameObject::removeRelatedFrame(int FrameID)
{
    return false;
}

bool FrameObject::removeAllRelatedFrames()
{
    return false;
}

FrameObject::RelatedFrameInfo::Ptr FrameObject::getRelatedFrame(int FrameID)
{
    return RelatedFrameInfo::Ptr();
}

bool FrameObject::getAllRelatedFrames(std::vector<RelatedFrameInfo::Ptr>& Frames)
{
    return false;
}

bool FrameObject::addMapPoint(int KeyPointID, std::shared_ptr<MapPointObject> MapPoint, const Eigen::Vector4d& LocalCoordinate)
{
    return false;
}

bool FrameObject::removeMapPoint(int KeyPointID)
{
    return false;
}

bool FrameObject::removeAllMapPoints()
{
    return false;
}

bool FrameObject::getMapPoint(int KeyPointID, std::shared_ptr<MapPointObject>& MapPoint, Eigen::Vector4d& LocalCoordinate)
{
    return false;
}

bool FrameObject::updateMapPoint(int KeyPointID, std::shared_ptr<MapPointObject> MapPoint, const Eigen::Vector4d& LocalCoordinate)
{
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
            int id = item.at("id");
            JsonNode pos = item.at("local-position");
            Eigen::Vector4d tmppos(pos.at(0), pos.at(1), pos.at(2), pos.at(3));
            ObservedMapPoints[id] = { std::weak_ptr<MapPointObject>(),tmppos }; //c++17的语法真高级
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
            this->RelatedFrame[tmpframe->getRelatedFrameID()] = tmpframe;
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
            {"type-id","frame-object"},
            {"id",this->FrameID},
            {"timestamp",this->Timestamp}
        };

        if (this->BestCamera != nullptr)
        {
            fs["best-camera-id"] = this->BestCamera->getRelatedFrameID();
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
            temp["id"] = item.first;
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

PinholeFrameObject::Ptr PinholeFrameObject::Create(int ID)
{
    return Ptr();
}

PinholeFrameObject::Ptr PinholeFrameObject::Create(int ID, const cv::Mat& CameraMatrix, const cv::Mat& DistCoeff, const cv::Mat& RGBMat, const cv::Mat& XYZMap)
{
    return Ptr();
}

PinholeFrameObject::PinholeFrameObject(int ID)
    :FrameObject(ID)
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
        fs["type-id"] = std::string("Pinhole-frame-object");
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

FrameObject::RelatedFrameInfo::Ptr FrameObject::RelatedFrameInfo::Create(std::shared_ptr<FrameObject> RelatedFrame)
{
    return RelatedFrameInfo::Ptr();
}

FrameObject::RelatedFrameInfo::Ptr FrameObject::RelatedFrameInfo::Create(std::shared_ptr<FrameObject> RelatedFrame, std::shared_ptr<Sophus::SE3d> Pose, double sigma)
{
    return RelatedFrameInfo::Ptr();
}

FrameObject::RelatedFrameInfo::Ptr FrameObject::RelatedFrameInfo::Create(std::shared_ptr<FrameObject> RelatedFrame, std::vector<cv::DMatch> KeyPointMatch)
{
    return RelatedFrameInfo::Ptr();
}

std::shared_ptr<FrameObject> FrameObject::RelatedFrameInfo::getRelatedFrame()
{
    return std::shared_ptr<FrameObject>();
}

int FrameObject::RelatedFrameInfo::getRelatedFrameID()
{
    return this->RelatedFrameID;
}

bool FrameObject::RelatedFrameInfo::isFrameExist()
{
    return false;
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
        fs["id"] = FramePtr->FrameID;
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
        this->RelatedFrameID = fs["id"].get<int>();
        return true;
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
    return false;
}

FrameObject::RelatedFrameInfo::RelatedFrameInfo()
    :sigma(0),
    RelatedFrameID(-1)
{

}

FrameObject::RelatedFrameInfo::RelatedFrameInfo(std::shared_ptr<FrameObject> RelatedFrame)
    :sigma(0),
    RelatedFrameID(-1)
{
}

FrameObject::RelatedFrameInfo::~RelatedFrameInfo()
{
}
