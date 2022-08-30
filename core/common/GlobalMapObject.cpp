#include "GlobalMapObject.h"

GlobalMapObject::Ptr GlobalMapObject::Create()
{
    return GlobalMapObject::Ptr();
}

GlobalMapObject::GlobalMapObject()
{
}

GlobalMapObject::~GlobalMapObject()
{
}

bool GlobalMapObject::getMaps(std::set<std::map<int, FrameObject>>& Frames, std::set<std::map<int, MapPointObject>>& MapPoint)
{
    return false;
}

bool GlobalMapObject::save(JsonNode& fs)
{
    return false;
}

bool GlobalMapObject::load(JsonNode& fs)
{
    return false;
}
