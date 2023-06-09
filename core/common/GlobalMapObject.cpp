﻿#include "GlobalMapObject.h"

GlobalMapObject::Ptr GlobalMapObject::Create()
{
	return std::make_shared<GlobalMapObject>();
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
	try
	{
		//save basic info
		fs["type-id"] = this->type_name();
		fs["frame-size"] = this->Frames.size();
		fs["mappoint-size"] = this->MapPoints.size();

		//save frame info
		JsonNode FramesNode = JsonNode::array();
		for (auto&& item : this, Frames)
		{
			JsonNode tmp = JsonNode::object();
			if (!item.second->save(tmp))
			{
				throw std::exception("failed to save frame, ID=" + item.first);
			}
			
			FramesNode.push_back(tmp);
		}
		fs["frame-info"] = FramesNode;

		//save mappoint info
		JsonNode MappointNode = JsonNode::array();
		for (auto&& item : this->MapPoints)
		{
			JsonNode tmp = JsonNode::object();
			if (!item.second->save(tmp))
			{
				throw std::exception("failed to save mappoint, ID=" + item.first);
			}
			MappointNode.push_back(tmp);
		}
		fs["mappoint-info"] = MappointNode;

		//save initialframe
		JsonNode InitialframesArray = JsonNode::array();
		for (auto& [key,value] : this->InitalFrames)
		{
			JsonNode tmp = JsonNode::object();
			tmp["map-id"] = key;
			tmp["initial-frame-id"] = value;
			InitialframesArray.push_back(tmp);
		}
		fs["initial-frames"] = InitialframesArray;
		
		return true;
	}
	catch (const std::exception& e)
	{
		fs = JsonNode::object();
		std::cerr << e.what() << std::endl;
	}
    return false;
}

bool GlobalMapObject::load(JsonNode& fs)
{
	try
	{
		//load basic parameters
		if (fs.at("type-id").get<std::string>() != this->type_name())
		{
			return false;
		}
		
		//load frames
		JsonNode FrameNode = fs.at("frame-info");
		for (auto&& Frame : FrameNode)
		{
			FrameObject::Ptr tmp = std::make_shared<FrameObject>();
			if (!tmp->load(Frame))
			{
				throw std::exception("failed to load frame");
			}
			this->Frames[tmp->getID()] = tmp;
		}

		//load mappoints
		JsonNode MappointNode = fs.at("mappoint-info");
		for (auto&& Mappoint : MappointNode)
		{
			MapPointObject::Ptr tmp = std::make_shared<MapPointObject>();
			if (!tmp->load(Mappoint))
			{
				throw std::exception("failed to load mappoint");
			}
			this->MapPoints[tmp->getID()] = tmp;
		}

		//load initial frames
		JsonNode InitialFrameNode = fs.at("initial-frames");
		for (auto&& Frame : InitialFrameNode)
		{
			this->InitalFrames.insert({ Frame.at("map-id"),Frame.at("initial-frame-id") });
		}

		//set related frames and observed mappoints
		for (auto&& item : this->Frames)
		{
			auto tmpframe = item.second;
			
			//fix shared state of mappoints
			for (size_t i = 0; i < tmpframe->KeyPoints.size(); i++)
			{
				int mappoint;
				Eigen::Vector4d position;
				if (tmpframe->getMapPoint(i, mappoint, position))
				{
					//reload with real shared mappoint
					tmpframe->updateMapPoint(i, this->MapPoints[mappoint], position);
				}
			}

			//fix shared state of related frames
			std::set<int> frameid;
			tmpframe->getAllRelatedFrames(frameid);
			for (auto&& id : frameid)
			{
				tmpframe->getRelatedFrame(id)->setRelatedFrame(this->Frames[id]);
			}
		}
		
		//set observed frames
		for (auto&& item : this->MapPoints)
		{
			auto tmppoint = item.second;
			std::set<int> frameid;
			tmppoint->getAllObservation(frameid);
			for (auto&& id : frameid)
			{
				tmppoint->updateObservation(this->Frames.at(id));
			}
		}
		
		return true;
	}
	catch (const std::exception& e)
	{
		this->MapPoints.clear();
		this->Frames.clear();
		std::cerr << e.what() << std::endl;
	}
    return false;
}

std::string GlobalMapObject::type_name()
{
	return std::string("global-map");
}


bool GlobalMapObject::addFrameObject(FrameObject::Ptr frame)
{
	if (this->Frames.count(frame->getID()) != 0)
		return false;

	this->Frames.insert(std::pair(frame->getID(), frame));
	return true;
}

bool GlobalMapObject::addMapPoint(MapPointObject::Ptr mappoint)
{
	if (this->Frames.count(mappoint->getID()) != 0)
		return false;

	this->MapPoints.insert(std::pair(mappoint->getID(), mappoint));
	return true;
}

bool GlobalMapObject::removeFrameObject(int ID)
{
	if (this->Frames.count(ID) == 0)
		return false;

	this->Frames.erase(ID);
	return true;
}

//TODO: 修复删除时删除路点引用的问题
bool GlobalMapObject::removeAllFrameObject()
{
	this->Frames.clear();
	return true;
}

bool GlobalMapObject::updateFrameObject(FrameObject::Ptr frame)
{
	if (this->Frames.count(frame->getID()) == 0)
		return false;

	this->Frames.at(frame->getID()) = frame;

	return true;
}

FrameObject::Ptr GlobalMapObject::getFrameObject(int ID)
{
	try
	{
		return this->Frames.at(ID);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	return FrameObject::Ptr();
}

bool GlobalMapObject::getAllFrameObjectID(std::set<int>& IDs)
{
	IDs.clear();
	for (auto& [key,value] : this->Frames)
	{
		IDs.insert(key);
	}
	return true;
}

//TODO: 修正关联问题
bool GlobalMapObject::updateMapPoint(MapPointObject::Ptr mappoint)
{
	if (this->MapPoints.count(mappoint->getID()) == 0)
		return false;
	this->MapPoints.at(mappoint->getID()) = mappoint;
	return true;
}

bool GlobalMapObject::removeMapPoint(int ID)
{
	if (this->MapPoints.count(ID) == 0)
		return false;
	this->MapPoints.erase(ID);
	return true;
}

bool GlobalMapObject::removeAllMapPoint()
{
	this->MapPoints.clear();
	return true;
}

MapPointObject::Ptr GlobalMapObject::getMapPoint(int ID)
{
	try
	{
		return this->MapPoints.at(ID);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	return nullptr;
}

bool GlobalMapObject::getAllMapPointID(std::set<int>& ID)
{
	ID.clear();
	for (auto& [key,value] : this->MapPoints)
	{
		ID.insert(key);
	}
	return true;
}

int GlobalMapObject::getFrameSize()
{
	return this->Frames.size();
}


int GlobalMapObject::getMappointSize()
{
	return this->MapPoints.size();
}

int GlobalMapObject::AssignMapID(int FrameID)
{
	this->InitalFrames.insert({ this->MapIDCounter,FrameID });
	return this->MapIDCounter++;
}

int GlobalMapObject::AssignMappointID()
{
	return this->MapPointCount++;
}

void GlobalMapObject::CleanUnusedMapID()
{
	std::set<int> MapID;
	for (auto& [key, value] : this->Frames)
	{
		MapID.insert(value->MapID);
	}

	for (auto it = this->InitalFrames.begin(); it != this->InitalFrames.end();)
	{
		if (MapID.count(it->first))
			++it;
		else
			it = this->InitalFrames.erase(it);
	}
}

std::ostream& operator<<(std::ostream& os, GlobalMapObject& obj)
{
	os << "GlobalMapObject" << std::endl;
	os << "Frame Size: " << obj.getFrameSize() << std::endl;
	os << "Mappoint Size: " << obj.getMappointSize() << std::endl;
	obj.CleanUnusedMapID();
	os << "Map Number: " << obj.InitalFrames.size() << std::endl;
	return os;
}
