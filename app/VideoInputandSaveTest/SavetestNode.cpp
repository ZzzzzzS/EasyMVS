#include "SavetestNode.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <nlohmann/json.hpp>

SavetestNode::SavetestNode(QObject *parent)
	: QObject(parent)
{
}

SavetestNode::~SavetestNode()
{
}

void SavetestNode::ShowWarning(std::string warning)
{
	std::cout << warning;
	std::ofstream save1("data.json");
	JsonNode savenode;
	this->map->save(savenode);
	save1 << std::setw(4) << savenode;
	save1.close();
}

void SavetestNode::tempshow(WorkFlowObject::DataQueue inputdata)
{
	auto ptr = std::dynamic_pointer_cast<PinholeFrameObject>(inputdata.front());
	std::cout << "Frame ID:" << ptr->getID() << std::endl;
	cv::Mat show;
	cv::drawKeypoints(ptr->RGBMat, ptr->KeyPoints, show);
	cv::imshow("Camera1", show);
	cv::waitKey(1);
	this->map->Frames.insert(std::make_pair(ptr->getID(), ptr));
}
