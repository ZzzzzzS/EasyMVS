#pragma once
#include <QWidget>
#include <QVTKOpenGLNativeWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <QMouseEvent>
#include <opencv2/opencv.hpp>


class QPointCloudViewer : public QVTKOpenGLNativeWidget
{
Q_OBJECT

public:
    QPointCloudViewer(QWidget *parent = nullptr);
    ~QPointCloudViewer();
    void AddPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const std::string& id, int viewport);
    pcl::visualization::PCLVisualizer::Ptr getViewerInstance();

signals:
    void ViewPointUpdated(cv::Mat Pose);

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

protected:
    void mouseReleaseEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

private:
    void ShowRightClickedMenu(QMouseEvent *event);

};
