#include "PoseReconstructor.h"

PoseReconstructor::Ptr PoseReconstructor::Create(GlobalMapObject::Ptr GlobalMap)
{
	return PoseReconstructor::Ptr();
}

PoseReconstructor::PoseReconstructor(GlobalMapObject::Ptr GlobalMap)
{
}

PoseReconstructor::~PoseReconstructor()
{
}

std::string PoseReconstructor::type_name()
{
	return std::string();
}

bool PoseReconstructor::clear()
{
	return false;
}

bool PoseReconstructor::init(JsonNode& fs)
{
	return false;
}

bool PoseReconstructor::saveParameter(JsonNode& fs)
{
	return false;
}

bool PoseReconstructor::Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
	return false;
}

bool PoseReconstructor::save(JsonNode& fs)
{
	return false;
}

bool PoseReconstructor::load(JsonNode& fs)
{
	return false;
}

void PoseReconstructor::Trigger()
{
}

void PoseReconstructor::Trigger(DataQueue data)
{
}
