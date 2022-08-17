#include "MapPointObject.h"

MapPointObject::Ptr MapPointObject::Create(int ID)
{
	return Ptr();
}

MapPointObject::MapPointObject(int ID)
	:ID(ID)
{
}

MapPointObject::~MapPointObject()
{
}

int MapPointObject::getFrameID()
{
	return this->ID;
}
