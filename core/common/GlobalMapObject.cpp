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

bool GlobalMapObject::save(JsonNode& fs)
{
    return false;
}

bool GlobalMapObject::load(JsonNode& fs)
{
    return false;
}
