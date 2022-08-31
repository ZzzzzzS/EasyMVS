#include "OptimalFrameFinder.h"

void OptimalFrameFinder::Trigger(DataQueue data)
{
}

bool OptimalFrameFinder::Compute(FrameObject::Ptr frame)
{
    return false;
}

bool OptimalFrameFinder::Compute(FrameObject::Ptr frame, std::vector<FrameObject::Ptr>& RelatedFrame)
{
    return false;
}

void OptimalFrameFinder::Trigger()
{
}

OptimalFrameFinder::Ptr OptimalFrameFinder::Create(GlobalMapObject::Ptr GlobalMap)
{
    return OptimalFrameFinder::Ptr();
}

OptimalFrameFinder::OptimalFrameFinder(GlobalMapObject::Ptr GlobalMap)
    :GlobalMap(GlobalMap)
{
}

OptimalFrameFinder::~OptimalFrameFinder()
{
}

bool OptimalFrameFinder::clear()
{
    return false;
}

bool OptimalFrameFinder::init(JsonNode& fs)
{
    return false;
}

bool OptimalFrameFinder::saveParameter(JsonNode& fs)
{
    return false;
}

bool OptimalFrameFinder::save(JsonNode& fs)
{
    return false;
}

bool OptimalFrameFinder::load(JsonNode& fs)
{
    return false;
}
