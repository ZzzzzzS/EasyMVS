#include <QObject>
#include <QCoreApplication>
#include <QApplication>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <iostream>
#include <QTimer>

#include "GlobalMapObject.h"
#include "FrameObject.h"
#include "MapPointObject.h"
#include "CameraObject.h"
#include "CameraModule/Photographer.h"
#include "FeatureExtraction/FeatureExtractor.h"
#include "SavetestNode.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    auto GlobalMapLoadTest = GlobalMapObject::Create();
    std::ifstream loader1("data.json");
    JsonNode loaderNode = JsonNode::parse(loader1);
    loader1.close();
    GlobalMapLoadTest->load(loaderNode);
    std::cout << "please manually check if load success." << std::endl;
    cv::waitKey(0);
    std::cout << "clear load" << std::endl;
    GlobalMapLoadTest.reset();

    //create global map and workflow
    auto GlobalMap = GlobalMapObject::Create();
    auto Camera1 = PinholeCamera::Create();
    auto Photographer1 = PinholePhotographer::Create({ Camera1 });
    auto FeatureExtractor1 = CVFeatureExtractor::Create();

    //load settings
    std::ifstream settings("test0.json");
    JsonNode Json = JsonNode::parse(settings);
    std::cout << Json;
    settings.close();
    if (JsonNode CameraNode = Json["Camera1"]; !Camera1->load(CameraNode))
    {
        std::cerr << "failed to load camera settings" << std::endl;
    }

    if (JsonNode PhotographerNode = Json["Photographer1"]; !Photographer1->load(PhotographerNode))
    {
        std::cerr << "failed to load photographer settings" << std::endl;
    }

    if (!Camera1->open())
    {
        std::cerr << "failed to open camera" << std::endl;
    }

	// load currently unimplemented
    if (auto ExtractorNode = Json["Extractor1"]; !FeatureExtractor1->load(ExtractorNode))
    {
        std::cerr << "failed to load extractor" << std::endl;
    }

    //connect signal-slots
    QTimer* t = new QTimer();
    QObject::connect(t, &QTimer::timeout, Photographer1.get(), static_cast<void (PinholePhotographer::*)(void)>(&PinholePhotographer::Trigger));

    QObject::connect(Photographer1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished), FeatureExtractor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&CVFeatureExtractor::Trigger));
	
    SavetestNode* t2 = new SavetestNode(nullptr);
    t2->map = GlobalMap;
    QObject::connect(FeatureExtractor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished), t2, &SavetestNode::tempshow);
    QObject::connect(Photographer1.get(), &WorkFlowObject::Warning, t2, &SavetestNode::ShowWarning);

    t->start(100);
    

    return a.exec();
}