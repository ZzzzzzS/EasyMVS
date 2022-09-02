#include "MapPointObject.h"
#include "JsonSaver.hpp"

MapPointObject::Ptr MapPointObject::Create(int ID)
{
	return Ptr();
}

MapPointObject::MapPointObject()
	:ID(-1),
	quality(0)
{
}

MapPointObject::MapPointObject(int ID)
	:ID(ID),
	quality(0)
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

bool MapPointObject::getObservation(int FrameID, std::shared_ptr<FrameObject>& Frame,int &KeyPointID)
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
	try
	{
		//save coeffcients
		fs["id"] = this->ID;
		fs["type-id"] = std::string("mappoint-object");
		fs["position"] = this->Position;
		fs["normal"] = this->Normal;
		fs["keypoint-descriptors"] = this->KeyPointDescriptor;
		fs["quality"] = this->quality;

		//save observation
		JsonNode observationnode = JsonNode::array();
		for (auto& item : this->ObservedFrame)
		{
			JsonNode tmp;
			tmp["frame-id"] = item.first;
			tmp["keypoint-id"] = std::get<1>(item.second);
			observationnode.push_back(tmp);
		}
		fs["observed-frame"] = observationnode;

		return true;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		fs = JsonNode::object();
	}
	return false;
}

bool MapPointObject::load(JsonNode& fs)
{
	try
	{
		//load coeffcients
		this->ID = fs.at("id").get<int>();
		this->Position = fs.at("position").get<Eigen::Vector4d>();
		this->Normal = fs.at("normal").get<Eigen::Vector3d>();
		this->KeyPointDescriptor = fs.at("keypoint-descriptors").get<cv::Mat>();
		this->quality = fs.at("quality").get<double>();

		//load observation
		JsonNode obsvnode = fs.at("observed-frame");
		for (auto& item : obsvnode)
		{
			int frameid = item.at("frame-id").get<int>();
			int keypointid = item.at("keypoint-id").get<int>();
			ObservedFrame[frameid] = { std::weak_ptr<FrameObject>(),keypointid };
		}

		return true;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
	}
	return false;
}
