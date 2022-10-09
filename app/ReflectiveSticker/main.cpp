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
#include "CameraModule/ImageReaderCamera.h"
#include "FeatureExtraction/ReflectiveStickerExtractor.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    auto GlobalMap = GlobalMapObject::Create();
    auto CameraLeft = PinholeImageReader::Create();
    auto CameraRight = PinholeImageReader::Create();
    auto Photographer1 = PinholePhotographer::Create(GlobalMap, { CameraLeft,CameraRight });
    auto FeatureExtractor1 = ReflectiveStickerExtractor::Create();

    //load settings
    std::ifstream settings("C:/Users/ZhouZishun/Documents/Workspace/CASIA_PROJECT/EasyMVS/app/ReflectiveSticker/test.json");
    JsonNode Json = JsonNode::parse(settings);
    settings.close();

    CameraLeft->load(Json["CameraLeft"]);
    CameraRight->load(Json["CameraRight"]);
    Photographer1->load(Json["Photographer1"]);
    FeatureExtractor1->load(Json["FeatureExtractor1"]);

    if (!CameraLeft->open())
    {
        std::cerr << "failed to open left camera" << std::endl;
    }
    if (!CameraRight->open())
    {
        std::cerr << "failed to open right camera" << std::endl;
    }

    std::vector<FrameObject::Ptr> frames = { PinholeFrameObject::Create(0,0),PinholeFrameObject::Create(1,0) };
	Photographer1->Compute(frames);

    FeatureExtractor1->Compute(frames[0]);
    FeatureExtractor1->Compute(frames[1]);


    return a.exec();
}