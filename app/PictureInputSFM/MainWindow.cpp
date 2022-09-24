#include "MainWindow.h"
#include "./ui_MainWindow.h"
#include "ScrollScale.h"
#include <iostream>
#include <sstream>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>
#include <QPushButton>
#include <opencv2/opencv.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Create Object
    this->GlobalMap = GlobalMapObject::Create();
    this->Camera1 = PinholeImageReader::Create();
    this->Photographer1 = PinholePhotographer::Create(this->GlobalMap, { Camera1 });
    this->Extractor1 = CVFeatureExtractor::Create();
    this->Matcher1 = VocTreeMatcher::Create(this->GlobalMap);
    this->PoseReconstructor1 = PinholePoseReconstructor::Create(this->GlobalMap);
	
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

    if (auto ExtractorNode = Json["Extractor1"]; !Extractor1->load(ExtractorNode))
    {
        std::cerr << "failed to load extractor" << std::endl;
    }

    if (auto MatcherNode = Json["Matcher1"]; !Matcher1->load(MatcherNode))
    {
        std::cerr << "failed to load matcher" << std::endl;
    }

    if (auto PoseReconstructorNode = Json["SFM1"]; !PoseReconstructor1->load(PoseReconstructorNode))
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
	Camera1->save(Json["Camera1"]);
	Photographer1->save(Json["Photographer1"]);
	Extractor1->save(Json["Extractor1"]);
	Matcher1->save(Json["Matcher1"]);
	PoseReconstructor1->save(Json["SFM1"]);
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
        auto relatedframe = relateinfo->getRelatedFrame();
        cv::Mat out;
		cv::drawMatches(frame->RGBMat, frame->KeyPoints, relatedframe->RGBMat, relatedframe->KeyPoints,
            relateinfo->KeyPointMatch,out);
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
    str << frame << std::endl;
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
    str << frame << std::endl;
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
    for (auto& i : FrameID)
    {
        this->ui->comboBox_FrameList->addItem(QString::number(i));
    }
	
	std::set<int> MappointID;
    this->GlobalMap->getAllMapPointID(MappointID);
    for (auto& i : MappointID)
    {
        this->ui->comboBox_MappointList->addItem(QString::number(i));
    }
	
    auto currentid = std::dynamic_pointer_cast<FrameObject>(data.front())->getID();
    this->FrameChangedSlot(QString::number(currentid));
}

