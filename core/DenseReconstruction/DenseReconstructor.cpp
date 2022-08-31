#include "DenseReconstructor.h"

DenseReconstructor::Ptr DenseReconstructor::Create(GlobalMapObject::Ptr GlobalMap)
{
    return DenseReconstructor::Ptr();
}

DenseReconstructor::DenseReconstructor(GlobalMapObject::Ptr GlobalMap)
    :GlobalMap(GlobalMap)
{
}

DenseReconstructor::~DenseReconstructor()
{
}

std::string DenseReconstructor::getFlowName()
{
    return std::string("Workflow DenseReconstructor");
}

bool DenseReconstructor::clear()
{
    return false;
}

bool DenseReconstructor::init(JsonNode& fs)
{
    return false;
}

bool DenseReconstructor::saveParameter(JsonNode& fs)
{
    return false;
}

bool DenseReconstructor::save(JsonNode& fs)
{
    return false;
}

bool DenseReconstructor::load(JsonNode& fs)
{
    return false;
}

bool DenseReconstructor::Compute(FrameObject::Ptr frame)
{
    return false;
}

bool DenseReconstructor::Compute(FrameObject::Ptr frame1, FrameObject Frame2)
{
    return false;
}

bool DenseReconstructor::Compute(std::vector<FrameObject::Ptr>& frames)
{
    return false;
}

void DenseReconstructor::Trigger()
{
}

void DenseReconstructor::Trigger(DataQueue data)
{
}
