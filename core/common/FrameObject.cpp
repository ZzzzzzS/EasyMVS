#include "FrameObject.h"

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
    return false;
}

bool FrameObject::save(JsonNode& fs)
{
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

bool FrameObject::RelatedFrameInfo::isFrameExist()
{
    return false;
}

bool FrameObject::RelatedFrameInfo::save(JsonNode& fs)
{
    return false;
}

bool FrameObject::RelatedFrameInfo::load(JsonNode& fs)
{
    return false;
}

FrameObject::RelatedFrameInfo::RelatedFrameInfo(std::shared_ptr<FrameObject> RelatedFrame)
    :sigma(0)
{
}

FrameObject::RelatedFrameInfo::~RelatedFrameInfo()
{
}
