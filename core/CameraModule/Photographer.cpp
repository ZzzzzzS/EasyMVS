#include "Photographer.h"

Photographer::Ptr Photographer::Create(std::initializer_list<CameraObject::Ptr> Cameras)
{
	return Photographer::Ptr();
}

Photographer::Photographer()
{
}

Photographer::Photographer(std::shared_ptr<CameraObject::Ptr> Camera)
{
}

Photographer::Photographer(std::initializer_list<CameraObject::Ptr> Cameras)
{
}

Photographer::~Photographer()
{
}

std::string Photographer::getFlowName()
{
	return std::string("Workflow Photographer");
}

bool Photographer::init(JsonNode& fs)
{
	return false;
}

bool Photographer::takePhoto(std::vector<FrameObject::Ptr>& Frames)
{
	return false;
}

bool Photographer::saveParameter(JsonNode& fs)
{
	return false;
}

bool Photographer::clear()
{
	return false;
}

void Photographer::Trigger(Photographer::DataQueue data)
{
}

void Photographer::Trigger()
{
	
}
