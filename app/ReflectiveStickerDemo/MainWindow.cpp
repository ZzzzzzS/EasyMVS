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
#include "common/JsonSaver.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	QObject::connect(this->ui->actionAboutQt, &QAction::triggered, qApp, &QApplication::aboutQt);
	QObject::connect(this->ui->actionLoad_Config, &QAction::triggered, this, &MainWindow::LoadConfigSlot);
    QObject::connect(this->ui->actionSave_Config, &QAction::triggered, this, &MainWindow::SaveConfigSlot);
    QObject::connect(this->ui->actionSave_Frame_Pose, &QAction::triggered, this, &MainWindow::SavePoseSlot);
	
    QObject::connect(this->ui->pushButton_ComputeNext, &QPushButton::clicked, this, &MainWindow::ComputeOnceSlot);

    cv::Mat1d LeftCamera(3, 3);
    LeftCamera << 1.6445922853787371e+04, 0, 1.2030723591663418e+03, 0,
        1.6437378215276756e+04, 1.0493475264763420e+03, 0, 0, 1;
	
	cv::Mat1d LeftDistcoeff(5,1);
    LeftDistcoeff << -2.7189247239838665e-01, 1.3516517868631182e+01,
        2.0034605898015576e-04, -8.4349183960178613e-04,
        1.5162508245217446e-01;
	
	cv::Mat1d RightCamera(3,3);
    RightCamera << 1.6334151688131926e+04, 0, 1.3061987706884240e+03, 0,
        1.6325895447275301e+04, 1.0726115849649595e+03, 0, 0, 1;
	
	cv::Mat1d RightDistcoeff(5,1);
    RightDistcoeff << -2.6047313279486806e-01, 1.3321850657748854e+01,
        -8.8479587125984722e-04, -3.0918403925120390e-04,
        1.1054610984412554e-01;
	
    cv::Mat1d TL2R(4,4);
    TL2R << 9.4371650323805911e-01, -6.0296128259525470e-03, -3.3070047669348690e-01, 1.5604369612221129e+02,
        7.1650333099359609e-03, 9.9997187887749017e-01, 2.2144416654232839e-03, 8.0206654659878107e-01,
        3.3067782479899954e-01, -4.4592850762384621e-03, 9.4373316724740097e-01, 1.6959180451950722e+01,
        0, 0, 0, 1;

    std::vector<std::string> Left = { "left_1.bmp","left_2.bmp","left_3.bmp","left_4.bmp" };
    std::vector<std::string> Right = { "right_1.bmp","right_2.bmp","right_3.bmp","right_4.bmp" };
    std::string path = "C:/Users/ZhouZishun/Documents/Workspace/CASIA_PROJECT/EasyMVS/app/ReflectiveSticker/data";
	
    std::vector<cv::Mat> LeftImg, RightImg;
	for (auto& l : Left)
	{
		LeftImg.push_back(cv::imread(path + "/" + l));
	}

    for (auto& r : Right)
    {
        RightImg.push_back(cv::imread(path + "/" + r));
    }
	

	
    this->core = new EasyMVSWarpper(LeftCamera, LeftDistcoeff, RightCamera, RightDistcoeff, TL2R);
	
    this->core->LoadImages(LeftImg, RightImg);
	
	QObject::connect(this->core, &EasyMVSWarpper::Finished, this, &MainWindow::FinishedSlot);
	
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::LoadConfigSlot()
{
    auto file = QFileDialog::getOpenFileName(this, tr("open file"), "./", "*.json");
	if (file.isEmpty())
	{
		return;
	}
	std::string path = file.toStdString();
    auto result = this->core->LoadParameters(path);
    if (result)
    {
        QMessageBox::information(this, "load", "load success");
    }
    else
    {
        QMessageBox::information(this, "load", "load failed");
    }
}


void MainWindow::SaveConfigSlot()
{
    auto file = QFileDialog::getSaveFileName(this, tr("save file"), "./", "*.json");
    if (file.isEmpty())
    {
        return;
    }
    std::string path = file.toStdString();
    auto result = this->core->SaveParameters(path);
    if (result)
    {
        QMessageBox::information(this, "save", "save success");
    }
    else
    {
        QMessageBox::information(this, "save", "save failed");
    }
}

void MainWindow::ComputeOnceSlot()
{
    this->core->Trigger();
}

void MainWindow::FinishedSlot(std::map<int, Sophus::SE3d> poses)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (auto& [id, pose] : poses)
	{
		std::cout << "id:" << id << "\npose:" << pose.matrix() << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr axis_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        cv::Mat1d MatPose;
        cv::eigen2cv(pose.matrix(), MatPose);
        this->DrawPointCloud(axis_cloud, MatPose);
		global->insert(global->end(), axis_cloud->begin(), axis_cloud->end());
	}
	this->ui->MapWidget->AddPointCloud(global, "test", 0);
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

//单独加一个保存位姿的函数，使用json的保存方法太长了没法看
void MainWindow::SavePoseSlot()
{
    auto file = QFileDialog::getSaveFileName(this, tr("save file"), "./", "*.json");
    if (file.isEmpty())
    {
        return;
    }
    std::string path = file.toStdString();
    auto result = this->core->SaveData(path);
	if (result)
	{
		QMessageBox::information(this, "save", "save success");
	}
	else
	{
		QMessageBox::information(this, "save", "save failed");
	}
}
