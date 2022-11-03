#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "ReflectiveStickerLib/EasyMVSWarpper.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
	
private: //slot
	void LoadConfigSlot();
	void SaveConfigSlot();
	void SavePoseSlot();

	void ComputeOnceSlot();

	void FinishedSlot(std::map<int, Sophus::SE3d> poses);

private:
    Ui::MainWindow *ui;
	
	EasyMVSWarpper* core;
	
	void DrawPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud, const cv::Mat1d& Pose);
};
#endif // MAINWINDOW_H
