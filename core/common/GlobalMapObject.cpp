#include "GlobalMapObject.h"

GlobalMapObject::Ptr GlobalMapObject::Create()
{
    return GlobalMapObject::Ptr();
}

GlobalMapObject::GlobalMapObject()
	:InitialFrameID(0)
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
		fs["initial-frame-id"] = this->InitialFrameID;

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
		
		this->InitialFrameID = fs.at("initial-frame-id").get<int>();
		
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

		//set related frames and observed mappoints
		for (auto&& item : this->Frames)
		{
			auto tmpframe = item.second;
			
			//fix shared state of mappoints
			for (size_t i = 0; i < tmpframe->KeyPoints.size(); i++)
			{
				int mappoint;
				Eigen::Vector4d position;
				tmpframe->getMapPoint(i, mappoint, position);
				//reload with real shared mappoint
				tmpframe->updateMapPoint(i, this->MapPoints[mappoint], position);
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
		this->InitialFrameID = -1;
		std::cerr << e.what() << std::endl;
	}
    return false;
}

std::string GlobalMapObject::type_name()
{
	return std::string("global-map");
}
