#include "VocTreeMatcher.h"

VocTreeMatcher::Ptr VocTreeMatcher::Create(GlobalMapObject::Ptr GlobalMap)
{
    return VocTreeMatcher::Ptr();
}

VocTreeMatcher::VocTreeMatcher(GlobalMapObject::Ptr GlobalMap)
    :FeatureMatcher(GlobalMap)
{
}

VocTreeMatcher::~VocTreeMatcher()
{
}

bool VocTreeMatcher::Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
    return false;
}

bool VocTreeMatcher::saveParameter(JsonNode& fs)
{
    return false;
}

bool VocTreeMatcher::init(JsonNode& fs)
{
    return false;
}

bool VocTreeMatcher::clear()
{
    return false;
}
