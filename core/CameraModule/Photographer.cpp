#include "Photographer.h"
#include "CameraObject.h"
#include "JsonSaver.hpp"
#include "FrameObject.h"

#include <sophus/se3.hpp>


Photographer::Ptr Photographer::Create(std::initializer_list<CameraObject::Ptr> Cameras)
{
	return std::make_shared<Photographer>(Cameras);
}

Photographer::Photographer()
{
}

Photographer::Photographer(CameraObject::Ptr Camera)
{
	//c++17的语法太先进了
	this->CamerasMap.emplace_back(Camera,Sophus::SE3d());
}

Photographer::Photographer(std::initializer_list<CameraObject::Ptr> Cameras)
{
	for (auto& Camera : Cameras)
	{
		this->CamerasMap.emplace_back(Camera, Sophus::SE3d());
	}
}

Photographer::~Photographer()
{
}

std::string Photographer::type_name()
{
	return std::string("workflow-photographer");
}

bool Photographer::init(JsonNode& fs)
{
	try
	{
		if(fs.at("type-id").get<std::string>()!=this->type_name())
		{
			return false;
		}
		auto extrinsics = fs.at("camera-extrinsics");

		int i = 0;
		for(auto&& item:extrinsics)
		{
			std::get<1>(this->CamerasMap.at(i))=item.get<Sophus::SE3d>();
			i++;
		}
		
		return true;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	return false;
}

bool Photographer::Compute(std::vector<FrameObject::Ptr>& Frames)
{
	if (this->CamerasMap.empty())
	{
		emit this->Error(this->type_name() + ": no camera found");
		return false;
	}
	
	if(Frames.size()!=this->CamerasMap.size())
	{
		emit this->Error(this->type_name()+": input frame size not match camera size!");
		return false;
	}

	try
	{
		int index=0;
		for(auto&& camera:this->CamerasMap)
		{
			auto CameraInstance = std::get<0>(camera);
			
			if (cv::Mat img;CameraInstance->read(img))
			{
				CameraInstance->undistort(img, img);
				Frames.at(index)->RGBMat = img;
			}
			else
			{
				emit this->Warning(this->type_name() + ": read frame failed!");
			}
			Frames.at(index)->setTimestamp(0);

			//set related frame
			if(index>=1) //multi camera senerio, related frame is needed
			{ 
				auto test=std::get<1>(camera);

				std::shared_ptr<Sophus::SE3d> se3ptr=std::make_shared<Sophus::SE3d>(std::get<1>(camera));

				auto frameinfo = FrameObject::RelatedFrameInfo::Create(Frames.at(index-1),se3ptr,0.0);
				if(!Frames.at(index)->addRelatedFrame(frameinfo))
				{
					emit this->Warning(this->type_name() + ": add related frame failed, frame id: "+std::to_string(Frames.at(index)->getID()));
				}
			}
			index++;
		}
		return true;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	
	return false;
}

bool Photographer::saveParameter(JsonNode& fs)
{
	try
	{
		fs["type-id"]=this->type_name();
		JsonNode extrinsics=JsonNode::array();
		for(auto&& item:this->CamerasMap)
		{
			extrinsics.push_back(std::get<1>(item));
		}
		fs["camera-extrinsics"]=extrinsics;
		return true;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	return false;
}

bool Photographer::clear()
{
	for (auto &&i : this->CamerasMap)
	{
		std::get<1>(i)= Sophus::SE3d();
		std::get<0>(i)->release();
	}
	this->m_isInit = false;
	return true;
}

bool Photographer::save(JsonNode& fs) 
{
	return false;
}

bool Photographer::load(JsonNode& fs) 
{
	return false;
}

void Photographer::Trigger(Photographer::DataQueue data)
{
	std::vector<FrameObject::Ptr> frames;
	frames.reserve(data.size());
	while (!data.empty())
	{
		FrameObject::Ptr frame=std::dynamic_pointer_cast<FrameObject>(data.front());
		if(frame!=nullptr)
		{
			frames.push_back(frame);
		}
		else
		{
			emit this->Warning(this->type_name() + ": dynamic cast input data error!");
		}
		data.pop();
	}
	
	if(this->Compute(frames))
	{
		emit this->Finished(data);
	}

}

void Photographer::Trigger()
{
	emit this->Warning(this->type_name() + ": this workflow can not be triggered without input data!");
}

PinholePhotographer::Ptr PinholePhotographer::Create(std::initializer_list<CameraObject::Ptr> Cameras)
{
	return std::make_shared<PinholePhotographer>(Cameras);
}

PinholePhotographer::PinholePhotographer()
{
}

PinholePhotographer::PinholePhotographer(std::initializer_list<CameraObject::Ptr> Cameras)
	:Photographer(Cameras)
{
	for (auto&& item : Cameras)
	{
		auto camera = std::dynamic_pointer_cast<PinholeCamera>(item);
		if (camera == nullptr)
		{
			emit this->Warning(this->type_name() + ": dynamic cast camera error, camera name=" + item->getCameraName());
		}
	}
}

PinholePhotographer::~PinholePhotographer()
{
}

bool PinholePhotographer::Compute(std::vector<FrameObject::Ptr>& Frames)
{
	if (!Photographer::Compute(Frames))
	{
		return false;
	}
	
	try
	{
		for (size_t i = 0; i < Frames.size(); i++)
		{
			auto tmpframe = std::dynamic_pointer_cast<PinholeFrameObject>(Frames[i]);
			if (tmpframe == nullptr)
			{
				emit this->Warning(this->type_name() + ": frame is not pinhole frame");
			}
			auto [CameraMatrix, distcoeff] = std::dynamic_pointer_cast<PinholeCamera>(std::get<CameraObject::Ptr>(this->CamerasMap.at(i)))->getCameraParameters();
			tmpframe->CameraMatrix = CameraMatrix;
			tmpframe->DistCoeff = distcoeff;
		}

		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	return false;
}

void PinholePhotographer::Trigger()
{
	DataQueue data;
	for (size_t i = 0; i < this->CamerasMap.size(); i++)
	{
		auto tmpframe = PinholeFrameObject::Create(this->FrameIDCounter++);
		data.push(tmpframe);
	}
	
	this->Trigger(data);
}

void PinholePhotographer::Trigger(Photographer::DataQueue data)
{
	std::vector<FrameObject::Ptr> frames;
	frames.reserve(data.size());
	while (!data.empty())
	{
		auto tmp = std::dynamic_pointer_cast<FrameObject>(data.front());
		if (tmp != nullptr)
		{
			frames.push_back(tmp);
		}
		else
		{
			emit this->Warning(this->type_name() + ": dynamic cast input data error!");
		}
	}

	if (this->Compute(frames))
	{
		emit this->Finished(data);
	}
}
