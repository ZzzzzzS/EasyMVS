﻿#include <QObject>
#include <QCoreApplication>
#include <QApplication>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <iostream>
#include <QTimer>
#include <QGridLayout>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>

#include "GlobalMapObject.h"
#include "FrameObject.h"
#include "MapPointObject.h"
#include "CameraObject.h"
#include "CameraModule/Photographer.h"
#include "FeatureExtraction/FeatureExtractor.h"
#include "CameraModule/ImageReaderCamera.h"
#include "FeatureExtraction/ReflectiveStickerExtractor.h"
#include "FeatureExtraction/ReflectiveStickerMatcher.h"
#include "PoseReconstruction/PinholePoseReconstructor.h"
#include "PoseReconstruction/ReflectiveReconstructor.h"
#include "visualization/QPointCloudViewer.h"
#include "MainWindow.h"



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
	MainWindow w;
	w.show();
	return a.exec();

    auto GlobalMap = GlobalMapObject::Create();
    auto CameraLeft = PinholeImageReader::Create();
    auto CameraRight = PinholeImageReader::Create();
    auto Photographer1 = PinholePhotographer::Create(GlobalMap, { CameraRight,CameraLeft });
    auto FeatureExtractor1 = ReflectiveStickerExtractor::Create();
	auto FeatureMatcher1 = ReflectiveStickerMatcher::Create(GlobalMap);
    auto PoseReconstructor1 = ReflectiveReconstructor::Create(GlobalMap);

    //load settings
    //std::ifstream settings("C:/Users/ZhouZishun/Documents/Workspace/CASIA_PROJECT/EasyMVS/app/ReflectiveSticker/test.json");
    std::ifstream settings("C:/Users/ZhouZishun/Desktop/test.json");
    JsonNode Json = JsonNode::parse(settings);
    settings.close();

    CameraLeft->load(Json["CameraLeft"]);
    CameraRight->load(Json["CameraRight"]);
    Photographer1->load(Json["Photographer1"]);
    FeatureExtractor1->load(Json["FeatureExtractor1"]);
	FeatureMatcher1->load(Json["FeatureMatcher1"]);
    PoseReconstructor1->load(Json["PoseReconstructor1"]);

    if (!CameraLeft->open())
    {
        std::cerr << "failed to open left camera" << std::endl;
    }
    if (!CameraRight->open())
    {
        std::cerr << "failed to open right camera" << std::endl;
    }

    std::vector<FrameObject::Ptr> frames = { PinholeFrameObject::Create(0,0),PinholeFrameObject::Create(1,0) };
    GlobalMap->addFrameObject(frames[0]);
    GlobalMap->addFrameObject(frames[1]);
	Photographer1->Compute(frames);
	//frame0 是右相机，frame1是左相机
    std::cout << "Frame 0 pose\n" << frames[0]->getGlobalPose().matrix() << std::endl;
    std::cout << "Frame 1 pose\n" << frames[1]->getGlobalPose().matrix() << std::endl;
	
    FeatureExtractor1->Compute(frames[0]);
    FeatureExtractor1->Compute(frames[1]);

    FeatureMatcher1->Compute(frames[1]);    
    PoseReconstructor1->Compute(frames[1]);
	
	std::vector<FrameObject::Ptr> frames2 = { PinholeFrameObject::Create(2,1),PinholeFrameObject::Create(3,1) };
    GlobalMap->addFrameObject(frames2[0]);
    GlobalMap->addFrameObject(frames2[1]);
    Photographer1->Compute(frames2);
    FeatureExtractor1->Compute(frames2[0]);
	FeatureExtractor1->Compute(frames2[1]);
    FeatureMatcher1->Compute(frames2[1]);
    PoseReconstructor1->Compute(frames2[1]);

	
    std::cout << *GlobalMap;
    std::set<int> MappointID;
    GlobalMap->getAllMapPointID(MappointID);
    pcl::PointCloud<pcl::PointXYZRGB> Mappoint;
    pcl::PointCloud<pcl::PointXYZRGB> FramePose;
    for (auto& i : MappointID)
    {
        std::cout << *(GlobalMap->getMapPoint(i)) << std::endl;
		auto point = GlobalMap->getMapPoint(i);
        pcl::PointXYZRGB tmp(point->Position(0), point->Position(1), point->Position(2), 255, 0, 0);
		Mappoint.push_back(tmp);
    }

    auto view = new QPointCloudViewer();
    view->AddPointCloud(Mappoint.makeShared(),"test",0);
    view->show();
	
    return a.exec();
}