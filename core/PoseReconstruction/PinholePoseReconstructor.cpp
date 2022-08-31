#include "PinholePoseReconstructor.h"

PinholePoseReconstructor::Ptr PinholePoseReconstructor::Create(GlobalMapObject::Ptr GlobalMap)
{
    return PinholePoseReconstructor::Ptr();
}

PinholePoseReconstructor::PinholePoseReconstructor(GlobalMapObject::Ptr GlobalMap)
    :PoseReconstructor(GlobalMap)
{
}

PinholePoseReconstructor::~PinholePoseReconstructor()
{
}

bool PinholePoseReconstructor::saveParameter(JsonNode& fs)
{
    return false;
}

bool PinholePoseReconstructor::init(JsonNode& fs)
{
    return false;
}

bool PinholePoseReconstructor::clear()
{
    return false;
}

bool PinholePoseReconstructor::Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
    return false;
}
