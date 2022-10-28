#include <QApplication>
#include "CameraModule/ImageReaderCamera.h"
#include "CameraModule/Photographer.h"
#include "GlobalMapObject.h"
#include "DenseReconstruction/DenseReconstructor.h"
#include "DenseReconstruction/DualFrameReconstructor.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>


int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
	
	auto GlobalMap1 = GlobalMapObject::Create();
	auto Camera1 = PinholeImageReader::Create();
	auto Camera2 = PinholeImageReader::Create();
	auto Photographer1 = PinholePhotographer::Create(GlobalMap1, { Camera1,Camera2 });
	auto DenseReconstructor1 = DualFrameReconstructor::Create(GlobalMap1);
	
	//load settings
	std::ifstream settings("C:/Users/ZhouZishun/Documents/Workspace/CASIA_PROJECT/EasyMVS/data/gxrdata/test.json");
	JsonNode Json = JsonNode::parse(settings);
	//std::cout << Json;
	settings.close();

	
	if (JsonNode CameraNode = Json["Camera1"]; !Camera1->load(CameraNode))
	{
		std::cerr << "failed to load camera settings" << std::endl;
	}

	if (JsonNode CameraNode = Json["Camera2"]; !Camera2->load(CameraNode))
	{
		std::cerr << "failed to load camera settings" << std::endl;
	}

	if (JsonNode PhotographerNode = Json["Photographer1"]; !Photographer1->load(PhotographerNode))
	{
		std::cerr << "failed to load photographer settings" << std::endl;
	}

	if (JsonNode DenseReconstructorNode = Json["DenseReconstructor1"]; !DenseReconstructor1->load(DenseReconstructorNode))
	{
		std::cerr << "failed to load DenseReconstructor settings" << std::endl;
	}

	if (!Camera1->open())
	{
		std::cerr << "failed to open camera" << std::endl;
	}

	if (!Camera2->open())
	{
		std::cerr << "failed to open camera" << std::endl;
	}

	std::vector<FrameObject::Ptr> frames = { PinholeFrameObject::Create(0,0),PinholeFrameObject::Create(1,0) };
	Photographer1->Compute(frames);
	frames[1]->setBestFrame(frames[1]->getRelatedFrame(0));
	
	DenseReconstructor1->Compute(frames[1]);
	
	pcl::PointCloud<pcl::PointXYZRGB> Cloud;
	//Cloud.reserve(frames[1]->XYZMat.cols * frames[1]->XYZMat.rows);
	for (int i = 0; i < frames[1]->XYZMat.cols; i++)
	{
		for (int j = 0; j < frames[1]->XYZMat.rows; j++)
		{
			if (frames[1]->XYZMat.at<cv::Vec3f>(j, i)[2] > 0)
			{
				pcl::PointXYZRGB Point;
				Point.x = frames[1]->XYZMat.at<cv::Vec3f>(j, i)[0];
				Point.y = frames[1]->XYZMat.at<cv::Vec3f>(j, i)[1];
				Point.z = frames[1]->XYZMat.at<cv::Vec3f>(j, i)[2];
				Point.r = frames[1]->RGBMat.at<cv::Vec3b>(j, i)[2];
				Point.g = frames[1]->RGBMat.at<cv::Vec3b>(j, i)[1];
				Point.b = frames[1]->RGBMat.at<cv::Vec3b>(j, i)[0];
				Cloud.push_back(Point);
			}
		}
	}

	pcl::PointCloud<pcl::PointXYZRGB> Cloudfilter;
	pcl::Indices index;
	pcl::removeNaNFromPointCloud(Cloud, Cloudfilter, index);
	
	//save pointcloud
	pcl::PLYWriter Writer;
	Writer.write("C:/Users/ZhouZishun/Documents/Workspace/CASIA_PROJECT/EasyMVS/data/gxrdata/DTUDense.ply", Cloudfilter);
	
	std::cout << "please manually check if rebuild success." << std::endl;

    return 0;
}