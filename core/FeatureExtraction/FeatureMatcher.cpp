#include "FeatureMatcher.h"

FeatureMatcher::Ptr FeatureMatcher::Create(GlobalMapObject::Ptr GlobalMap)
{
    return FeatureMatcher::Ptr();
}

FeatureMatcher::FeatureMatcher(GlobalMapObject::Ptr GlobalMap)
{
}

FeatureMatcher::~FeatureMatcher()
{
}

std::string FeatureMatcher::type_name()
{
    return std::string("Workflow FeatureMatcher");
}

bool FeatureMatcher::clear()
{
    return false;
}

bool FeatureMatcher::Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
    return false;
}

bool FeatureMatcher::save(JsonNode& fs)
{
    return false;
}

bool FeatureMatcher::load(JsonNode& fs)
{
    return false;
}

void FeatureMatcher::Trigger()
{
}

void FeatureMatcher::Trigger(DataQueue data)
{
}
