#include "MainWindow.h"
#include "./ui_MainWindow.h"
#include "visualization/ScrollScale.h"
#include <iostream>
#include <sstream>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>
#include <QPushButton>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/common/transforms.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Create Object
    this->GlobalMap = GlobalMapObject::Create();
    this->Camera1 = PinholeImageReader::Create();
    this->Camera2 = PinholeImageReader::Create();
    this->Photographer1 = PinholePhotographer::Create(this->GlobalMap, { Camera1, Camera2 });
    this->Extractor1 = ReflectiveStickerExtractor::Create();
    this->Matcher1 = ReflectiveStickerMatcher::Create(GlobalMap);
    this->PoseReconstructor1 = ReflectiveReconstructor::Create(GlobalMap);
	
	QObject::connect(this->ui->actionAboutQt, &QAction::triggered, qApp, &QApplication::aboutQt);
	QObject::connect(this->ui->actionLoad_Config, &QAction::triggered, this, &MainWindow::LoadConfigSlot);
    QObject::connect(this->ui->actionSave_Config, &QAction::triggered, this, &MainWindow::SaveConfigSlot);
    QObject::connect(this->ui->actionSave_Data, &QAction::triggered, this, &MainWindow::SaveDataSlot);
	
    QObject::connect(this->ui->pushButton_ComputeNext, &QPushButton::clicked, this, &MainWindow::ComputeOnceSlot);
    QObject::connect(this->ui->comboBox_CurrentMatch, &QComboBox::currentTextChanged, this, &MainWindow::RelatedFrameChangedSlot);
    QObject::connect(this->ui->comboBox_MappointList, &QComboBox::currentTextChanged, this, &MainWindow::MappointChangedSlot);
    QObject::connect(this->ui->comboBox_FrameList, &QComboBox::currentTextChanged, this, &MainWindow::FrameChangedSlot);
	
    QObject::connect(this->Extractor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished), this, [this](WorkFlowObject::DataQueue in) {
        auto frame = std::dynamic_pointer_cast<FrameObject>(in.front());
        cv::Mat out;
        cv::drawKeypoints(frame->RGBMat, frame->KeyPoints, out);
        this->ui->label_CurrentImage->LoadPictureAsyn(out);
        });
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::InitSystem(std::string path)
{
    std::ifstream settings(path);
    JsonNode Json = JsonNode::parse(settings);
    settings.close();

	
	//config Object
    if (JsonNode CameraNode = Json["CameraRight"]; !Camera1->load(CameraNode))
    {
        std::cerr << "failed to load camera settings" << std::endl;
    }
	
    if (JsonNode CameraNode = Json["CameraLeft"]; !Camera2->load(CameraNode))
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

    if (!Camera2->open())
    {
        std::cerr << "failed to open camera" << std::endl;
    }
	
    if (auto ExtractorNode = Json["FeatureExtractor1"]; !Extractor1->load(ExtractorNode))
    {
        std::cerr << "failed to load extractor" << std::endl;
    }

    if (auto MatcherNode = Json["FeatureMatcher1"]; !Matcher1->load(MatcherNode))
    {
        std::cerr << "failed to load matcher" << std::endl;
    }

    if (auto PoseReconstructorNode = Json["PoseReconstructor1"]; !PoseReconstructor1->load(PoseReconstructorNode))
    {
		std::cerr << "failed to load PoseReconstructor" << std::endl;
    }

	//Init signal-slot
    QObject::connect(this->ui->pushButton_ComputeNext,&QPushButton::clicked, this, [this]() {
        this->Photographer1->Trigger();
        });
	
    QObject::connect(Photographer1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        Extractor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&CVFeatureExtractor::Trigger));
	
    QObject::connect(Extractor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        Matcher1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&VocTreeMatcher::Trigger));
	
    QObject::connect(Matcher1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        PoseReconstructor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&PoseReconstructor::Trigger));
	
    QObject::connect(PoseReconstructor1.get(), static_cast<void (WorkFlowObject::*)(WorkFlowObject::DataQueue)>(&WorkFlowObject::Finished),
        this, &MainWindow::UpdateMap);
	
    this->isInit = true;
}

void MainWindow::SaveSystem(std::string path)
{
    MVSConfig::GlobalConfig::WorkspacePath = path;
	JsonNode Json;
	Camera1->save(Json["CameraLeft"]);
    Camera2->save(Json["CameraRight"]);
	Photographer1->save(Json["Photographer1"]);
	Extractor1->save(Json["FeatureExtractor1"]);
	Matcher1->save(Json["FeatureMatcher1"]);
	PoseReconstructor1->save(Json["PoseReconstructor1"]);
	std::ofstream settings(path);
    settings << std::setw(4) << Json;
	settings.close();
}

void MainWindow::SaveData(std::string path)
{
    MVSConfig::GlobalConfig::WorkspacePath = path;
	std::ofstream data(path);
    JsonNode Json;
    this->GlobalMap->save(Json);
	data << std::setw(4)  << Json;
	data.close();
}

void MainWindow::LoadConfigSlot()
{
    auto file = QFileDialog::getOpenFileName(this, tr("open file"), "./", "*.json");
	if (file.isEmpty())
	{
		return;
	}
	std::string path = file.toStdString();
	if (!isInit)
	{
        InitSystem(path);
	}
}

void MainWindow::SaveDataSlot()
{
	auto file = QFileDialog::getSaveFileName(this, tr("save file"), "./", "*.json");
	if (file.isEmpty())
	{
		return;
	}
	std::string path = file.toStdString();
    SaveData(path);
}

void MainWindow::SaveConfigSlot()
{       
	auto file = QFileDialog::getSaveFileName(this, tr("save file"), "./", "*.json");
	if (file.isEmpty())
	{
		return;
	}
	std::string path = file.toStdString();
	SaveSystem(path);
}

void MainWindow::ComputeOnceSlot()
{
}

void MainWindow::RelatedFrameChangedSlot(QString index)
{
    bool ok;
    int i = index.toInt(&ok);
    if (ok)
    {
        bool p = false;
        int number = this->ui->comboBox_FrameList->currentText().toInt(&p);
        if (!p)
            return;
		
        auto frame = this->GlobalMap->getFrameObject(number);
        
		
        auto relateinfo = frame->getRelatedFrame(i);
        if (relateinfo == nullptr)
            return;
        auto relatedframe = relateinfo->getRelatedFrame();
        cv::Mat out;
		cv::drawMatches(frame->RGBMat, frame->KeyPoints, relatedframe->RGBMat, relatedframe->KeyPoints,
            relateinfo->KeyPointMatch,out);
        cv::resize(out, out, cv::Size(), 0.5, 0.5);
		cv::imshow("related", out);
        ScrollScale* show = new ScrollScale(Q_NULLPTR);
        show->setAttribute(Qt::WA_DeleteOnClose);
        show->LoadPictureAsyn(out);
    }
    else
    {
        return;
    }
}

void MainWindow::FrameChangedSlot(QString index)
{
    bool ok;
    int i = index.toInt(&ok);
    if (ok)
    {
        this->ui->textEdit_FrameInfo->clear();
    }
    else
	{
		return;
	}
    auto frame = this->GlobalMap->getFrameObject(i);
    
    std::stringstream str;
    str << *frame << std::endl;
    std::string info = str.str();
    this->ui->textEdit_FrameInfo->setText(QString::fromStdString(info));

    std::set<int> ids;
    frame->getAllRelatedFrames(ids);
    this->ui->comboBox_CurrentMatch->clear();
    for (auto id : ids)
    {
        this->ui->comboBox_CurrentMatch->addItem(QString::number(id));
    }
}

void MainWindow::MappointChangedSlot(QString index)
{
    bool ok;
    int i = index.toInt(&ok);
    if (ok)
    {
        this->ui->textEdit_MappointInfo->clear();

    }
    auto frame = this->GlobalMap->getMapPoint(i);

    std::stringstream str;
    str << *frame << std::endl;
    std::string info = str.str();
    this->ui->textEdit_MappointInfo->setText(QString::fromStdString(info));
}


void MainWindow::UpdateMap(WorkFlowObject::DataQueue data)
{
    this->ui->comboBox_CurrentMatch->clear();
    this->ui->comboBox_FrameList->clear();
    this->ui->comboBox_MappointList->clear();
	
    std::set<int> FrameID;
	this->GlobalMap->getAllFrameObjectID(FrameID);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudAll(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto& i : FrameID)
    {
        this->ui->comboBox_FrameList->addItem(QString::number(i));
		
        auto frame = this->GlobalMap->getFrameObject(i);
        cv::Mat1d MatPose;
        DataFlowObject::Sophus2cvMat(frame->getGlobalPose(), MatPose);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        this->DrawPointCloud(cloud, MatPose);
        //this->ui->MapWidget->AddPointCloud(cloud,"test",0);
		cloudAll->insert(cloudAll->end(), cloud->begin(), cloud->end());
    }
	
	std::set<int> MappointID;
    this->GlobalMap->getAllMapPointID(MappointID);
    for (auto& i : MappointID)
    {
        this->ui->comboBox_MappointList->addItem(QString::number(i));
		auto mappoint = this->GlobalMap->getMapPoint(i);
		pcl::PointXYZRGB point;
        point.x = mappoint->Position(0);
		point.y = mappoint->Position(1);
		point.z = mappoint->Position(2);
		point.r = 255;
		point.g = 0;
		point.b = 0;
        cloudAll->push_back(point);
    }
    this->ui->MapWidget->AddPointCloud(cloudAll, "test", 0);
	

    auto currentid = std::dynamic_pointer_cast<FrameObject>(data.front())->getID();
    this->FrameChangedSlot(QString::number(currentid));
}


//公老板赞助的非常先进的点云生成函数
void MainWindow::DrawPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud, const cv::Mat1d& Pose)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr axis_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    float axis_pixLength = 0.1;//轴横向像素点间隔
    float axis_pixWidth = 1;//轴宽度像素点间隔

    int axis_pixLengthNum = 1000;//轴横向像素个数
    int axis_pixWidthNum = 6;//轴像素宽度像素个数

    for (int i = 0; i < axis_pixLengthNum; i++)
    {
        pcl::PointXYZRGB uu;
        for (int m = 0; m < axis_pixWidthNum; m++)
        {
            for (int n = 0; n < axis_pixWidthNum; n++)
            {
                float i_axis = i * axis_pixLength;
                float n_axis = n * axis_pixWidth;
                float m_axis = m * axis_pixWidth;

                // 添加x轴
                uu.x = i_axis;
                uu.y = n_axis;
                uu.z = m_axis;
                uu.r = 255;
                uu.g = 0;
                uu.b = 0;
                axis_cloud->points.push_back(uu);

                // 添加y轴
                uu.y = i_axis;
                uu.x = n_axis;
                uu.z = m_axis;
                uu.r = 0;
                uu.g = 255;
                uu.b = 0;
                axis_cloud->points.push_back(uu);
                // 添加z轴
                uu.z = i_axis;
                uu.x = n_axis;
                uu.y = m_axis;
                uu.r = 0;
                uu.g = 0;
                uu.b = 255;
                axis_cloud->points.push_back(uu);
            }
        }
    }

    cv::Mat1d rot = cv::Mat1d::eye(4, 4);

    Eigen::Matrix4d EigenPose;
    cv::cv2eigen(rot * Pose, EigenPose);
    Eigen::Affine3d EigenMat(EigenPose);
    pcl::transformPointCloud(*axis_cloud, *axis_cloud, EigenMat);
    PointCloud->insert(PointCloud->end(), axis_cloud->begin(), axis_cloud->end());
}