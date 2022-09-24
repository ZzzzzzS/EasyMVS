#include "QPointCloudViewer.h"
#include <QWidget>
#include <QVTKOpenGLNativeWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <QProgressDialog>
#include <QApplication>
#include <vtkOutputWindow.h>
#include <QDebug>
#include <iostream>
#include <QMenu>
#include <opencv2/core/eigen.hpp>
#include <QAction>
#include <QFileDialog>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/common/common.h>
#include <QMessageBox>


QPointCloudViewer::QPointCloudViewer(QWidget *parent)
    : QVTKOpenGLNativeWidget(parent)
{
	qRegisterMetaType<cv::Mat>("MyClass");

	vtkOutputWindow::SetGlobalWarningDisplay(0);//不弹出vtkOutputWindow窗口
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(0.0, 0.0, 0.0);
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    this->viewer=pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(renderer, renderWindow,"viewer", false));
    this->setRenderWindow(viewer->getRenderWindow());
    this->viewer->setupInteractor(this->interactor(), this->renderWindow());

    this->viewer->setBackgroundColor(0.3411764705882353, 0.3411764705882353, 0.3411764705882353);
    this->viewer->addCoordinateSystem();
    this->update();
}

QPointCloudViewer::~QPointCloudViewer()
{
}

void QPointCloudViewer::AddPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const std::string& id, int viewport)
{
	QProgressDialog* Progress = new QProgressDialog(this);
	Progress->setLabelText(tr("Loading PointCloud"));
	Progress->setMinimum(0);
	Progress->setMaximum(100);
	Progress->show();
	qApp->processEvents();
	Progress->setValue(0);
	this->viewer->removeAllPointClouds(viewport);
	Progress->setValue(20);
	this->viewer->removeAllCoordinateSystems(viewport);
	Progress->setValue(40);
	qApp->processEvents();
	this->viewer->addPointCloud(cloud, id, viewport);
	Progress->setValue(60);
	qApp->processEvents();
	this->viewer->updatePointCloud(cloud, id);
	this->viewer->addCoordinateSystem();
	Progress->setValue(80);
	this->update();
	qApp->processEvents();
	Progress->setValue(100);
	Progress->close();
}

pcl::visualization::PCLVisualizer::Ptr QPointCloudViewer::getViewerInstance()
{
	return this->viewer;
}

void QPointCloudViewer::mouseReleaseEvent(QMouseEvent *event)
{
	if(event->button()==Qt::RightButton)
	{
		this->ShowRightClickedMenu(event);
	}
	else if(event->button()==Qt::MouseButton::MiddleButton || event->button()==Qt::MouseButton::LeftButton)
	{
		//qDebug()<<"mouseReleaseEvent";
		Eigen::Affine3f  pos=this->viewer->getViewerPose();
		//std::cout<<"Camera Pose\n" << pos.matrix() << std::endl;
		cv::Mat Pose;
		cv::eigen2cv(pos.matrix(), Pose);
		emit ViewPointUpdated(Pose);
	}

	std::vector<pcl::visualization::Camera> cameras;
	this->viewer->getCameras(cameras);
}

void QPointCloudViewer::wheelEvent(QWheelEvent *event)
{
	Eigen::Affine3f  pos=this->viewer->getViewerPose();
	//std::cout<< pos.matrix()<<std::endl;
	cv::Mat Pose;
	cv::eigen2cv(pos.matrix(), Pose);
	emit ViewPointUpdated(Pose);
}

void QPointCloudViewer::ShowRightClickedMenu(QMouseEvent *event)
{
	QMenu* menu = new QMenu(this);
    /**********************************************************************/
	QAction* action_loadPointCloud = new QAction(tr("Load Point Cloud"), menu);
	menu->addAction(action_loadPointCloud);
	QObject::connect(action_loadPointCloud, &QAction::triggered, [=]() {
		QString FileName = QFileDialog::getOpenFileName(this, tr("Select Point Cloud File"), "./", "PointCloud Files (*.ply)");
		if (FileName.isEmpty())
			return;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::io::loadPLYFile<pcl::PointXYZRGB>(FileName.toStdString(), *temp);
		this->AddPointCloud(temp, "cloud", 0);
	});

    /**********************************************************************/
    QAction* action_resetview = new QAction(tr("reset view"),menu);
    
    menu->addAction(action_resetview);
	QObject::connect(action_resetview, &QAction::triggered, [=](){
		this->viewer->setCameraPosition(0.5, 0.5, 4, 0, 0, 0);
	});
	
    menu->addSeparator();
    /**********************************************************************/
    QAction* action_save_img = new QAction(tr("save as image"),menu);

    menu->addAction(action_save_img);
	QObject::connect(action_save_img, &QAction::triggered, [=](){
		QString filename = QFileDialog::getSaveFileName(this, tr("Save Image"), "./", "Image Files (*.png *.jpg *.bmp)");
		if(!filename.isEmpty())
		{
			this->viewer->saveScreenshot(filename.toStdString());
			QMessageBox::information(this, tr("Save Data"), tr("Save Success!"));
		}
	});

    /**********************************************************************/
    QAction* action_save_data=new QAction(tr("save data"),menu);
    QObject::connect(action_save_data, &QAction::triggered, [=](){
		QString filename = QFileDialog::getSaveFileName(this, tr("Save Data"), "./", "Data Files (*.txt)");
		if(!filename.isEmpty())
		{
			std::ofstream out(filename.toStdString());
			out<<this->viewer->getViewerPose().matrix()<<std::endl;
			out.close();
		}
		QMessageBox::information(this, tr("Save Data"), tr("Unsupported Function!"));
	});


    menu->addAction(action_save_data);

#if QT_VERSION_MAJOR ==6
    menu->exec(event->globalPosition().toPoint());
#else
    menu->exec(event->globalPos());
#endif
    delete menu;
}
