#include "DualFrameReconstructor.h"

DualFrameReconstructor::Ptr DualFrameReconstructor::Create(GlobalMapObject::Ptr GlobalMap)
{
    return DualFrameReconstructor::Ptr();
}

DualFrameReconstructor::DualFrameReconstructor(GlobalMapObject::Ptr GlobalMap)
    :DenseReconstructor(GlobalMap)
{
}

DualFrameReconstructor::~DualFrameReconstructor()
{
}

bool DualFrameReconstructor::clear()
{
    return false;
}

bool DualFrameReconstructor::Compute(FrameObject::Ptr frame)
{
    return false;
}

bool DualFrameReconstructor::Compute(FrameObject::Ptr frame1, FrameObject Frame2)
{
    return false;
}
