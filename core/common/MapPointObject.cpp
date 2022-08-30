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

int MapPointObject::getObservedTimes()
{
	return 0;
}

bool MapPointObject::addObservation(std::shared_ptr<FrameObject> Frame, int KeyPointID)
{
	return false;
}

bool MapPointObject::removeObservation(std::shared_ptr<FrameObject> Frame)
{
	return false;
}

bool MapPointObject::updateObservation(std::shared_ptr<FrameObject> Frame, int KeyPointID)
{
	return false;
}

bool MapPointObject::getAllObservation(std::vector<std::shared_ptr<FrameObject>>& Frames)
{
	return false;
}

bool MapPointObject::setMapPointQuality(double quality)
{
	return false;
}

double MapPointObject::getMapPointQuality()
{
	return 0.0;
}

bool MapPointObject::save(JsonNode& fs)
{
	return false;
}

bool MapPointObject::load(JsonNode& fs)
{
	return false;
}
