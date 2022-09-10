#include "DualPhotographer.h"

DualPhotographer::Ptr DualPhotographer::Create(std::initializer_list<CameraObject::Ptr> Cameras)
{
    return DualPhotographer::Ptr();
}

DualPhotographer::DualPhotographer()
{
}

DualPhotographer::DualPhotographer(std::initializer_list<CameraObject::Ptr> Cameras)
{
}

DualPhotographer::~DualPhotographer()
{
}

bool DualPhotographer::Compute(std::vector<FrameObject::Ptr>& Frames)
{
    return false;
}

bool DualPhotographer::clear()
{
    return false;
}
