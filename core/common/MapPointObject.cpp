#include "MapPointObject.h"
#include "JsonSaver.hpp"
#include "FrameObject.h"

MapPointObject::Ptr MapPointObject::Create(int ID)
{
	return std::make_shared<MapPointObject>(ID);
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

int MapPointObject::getID()
{
	return this->ID;
}

int MapPointObject::getObservedTimes()
{
	return	this->ObservedFrame.size();
}

bool MapPointObject::addObservation(std::shared_ptr<FrameObject> Frame, int KeyPointID)
{
	try
	{
		//syntex from c++ 17
		if (int id = Frame->getID(); this->ObservedFrame.count(id) != 0)
		{
			return false;
		}

		this->ObservedFrame[Frame->getID()] = { Frame,KeyPointID };
		
		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	return false;
}

bool MapPointObject::removeObservation(std::shared_ptr<FrameObject> Frame)
{
	try
	{
		if (auto id = Frame->getID(); this->ObservedFrame.count(id) == 0)
		{
			return false;
		}

		this->ObservedFrame.erase(Frame->getID());
		
		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	return false;
}

bool MapPointObject::updateObservation(std::shared_ptr<FrameObject> Frame, int KeyPointID)
{
	try
	{
		auto id = Frame->getID();
		if (this->ObservedFrame.count(id) == 0)
		{
			return false;
		}
		
		std::get<0>(this->ObservedFrame[id]) = Frame;
		if (KeyPointID != -1) //won't update keypointid when it equals -1
		{
			std::get<1>(this->ObservedFrame[id]) = KeyPointID;
		}
		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	return false;
}

bool MapPointObject::getObservation(int FrameID, std::shared_ptr<FrameObject>& Frame,int &KeyPointID)
{
	if (this->ObservedFrame.count(FrameID) == 0)
	{
		return false;
	}
	
	if (Frame = std::get<0>(this->ObservedFrame.at(FrameID)).lock();Frame == nullptr)
	{
		return false;
	}
	
	KeyPointID = std::get<1>(this->ObservedFrame.at(FrameID));
	return true;
}

bool MapPointObject::getAllObservation(std::set<std::shared_ptr<FrameObject>>& Frames)
{
	Frames.clear();
	if (this->ObservedFrame.size() == 0)
	{
		return false;
	}
	
	for (auto&& item : this->ObservedFrame)
	{
		Frames.insert(std::get<0>(item.second).lock());
	}
	return true;
}

bool MapPointObject::getAllObservation(std::set<int>& ids)
{
	if (this->ObservedFrame.empty())
		return false;

	for (auto&& item : this->ObservedFrame)
	{
		ids.insert(item.first);
	}
}

bool MapPointObject::setMapPointQuality(double quality)
{
	if (quality >= 0)
	{
		this->quality = quality;
		return true;
	}
	else
	{
		return false;
	}
}

double MapPointObject::getMapPointQuality()
{
	return this->quality;
}

bool MapPointObject::save(JsonNode& fs)
{
	try
	{
		//save coeffcients
		fs["id"] = this->ID;
		fs["type-id"] = this->type_name();
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
		std::cerr << e.what() << std::endl;
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
		std::cerr << e.what() << std::endl;
	}
	return false;
}

std::string MapPointObject::type_name()
{
	return std::string("mappoint-object");
}

int MapPointObject::assignNewMapID()
{
	return this->MapCounter++;
}