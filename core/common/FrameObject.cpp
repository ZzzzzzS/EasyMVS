#include "FrameObject.h"

FrameObject::Ptr FrameObject::Create(int ID)
{
    return Ptr();
}

FrameObject::Ptr FrameObject::Create(int ID, const cv::Mat& RGBMat, const cv::Mat& XYZMap)
{
    return Ptr();
}

FrameObject::FrameObject(int ID)
    :FrameID(ID)
{
}

FrameObject::~FrameObject()
{
}

int FrameObject::getFrameID()
{
    return this->FrameID;
}

bool FrameObject::getBestCamera(FrameObject::Ptr& Camera, Sophus::SE3d& Pose)
{
    return false;
}

bool FrameObject::setBestCamera(FrameObject::Ptr Camera, const Sophus::SE3d Pose)
{
    return false;
}

bool FrameObject::addRelatedFrame(FrameObject::Ptr Frame, const Sophus::SE3d Pose)
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

FrameObject::Ptr FrameObject::getRelatedFrame(int FrameID)
{
    return FrameObject::Ptr();
}

bool FrameObject::getAllRelatedFrames(std::vector<FrameObject::Ptr>& Frames)
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
