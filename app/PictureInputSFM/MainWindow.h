#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "CameraModule/ImageReaderCamera.h"
#include "CameraModule/Photographer.h"
#include "FrameObject.h"
#include "MapPointObject.h"
#include "GlobalMapObject.h"
#include "CameraObject.h"

#include "FeatureExtraction/FeatureExtractor.h"
#include "FeatureExtraction/VocTreeMatcher.h"
#include "FeatureExtraction/FSift128.h"
#include "PoseReconstruction/PinholePoseReconstructor.h"



QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
	void InitSystem(std::string path);
	void SaveSystem(std::string path);
	void SaveData(std::string path);
	
private: //slot
	void LoadConfigSlot();
	void SaveDataSlot();
	void SaveConfigSlot();

	void ComputeOnceSlot();
	void RelatedFrameChangedSlot(QString index);
	void FrameChangedSlot(QString index);
	void MappointChangedSlot(QString index);

	void UpdateMap(WorkFlowObject::DataQueue data);
private:
	CameraObject::Ptr Camera1;
	Photographer::Ptr Photographer1;
	FeatureExtractor::Ptr Extractor1;
	FeatureMatcher::Ptr Matcher1;
	PoseReconstructor::Ptr PoseReconstructor1;

	GlobalMapObject::Ptr GlobalMap;
	
private:
    Ui::MainWindow *ui;
	bool isInit = false;
};
#endif // MAINWINDOW_H
