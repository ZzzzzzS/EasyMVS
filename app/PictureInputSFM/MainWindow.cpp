#include "MainWindow.h"
#include "./ui_MainWindow.h"
#include <iostream>
#include <sstream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
	QObject::connect(this->ui->actionAboutQt, &QAction::triggered, qApp, &QApplication::aboutQt);
	QObject::connect(this->ui->actionLoad_Config, &QAction::triggered, this, &MainWindow::LoadConfigSlot);
    QObject::connect(this->ui->actionSave_Config, &QAction::triggered, this, &MainWindow::SaveConfigSlot);
    QObject::connect(this->ui->actionSave_Data, &QAction::triggered, this, &MainWindow::SaveDataSlot);
	
    QObject::connect(this->ui->pushButton_ComputeNext, &QPushButton::clicked, this, &MainWindow::ComputeOnceSlot);
    QObject::connect(this->ui->comboBox_CurrentMatch, &QComboBox::currentTextChanged, this, &MainWindow::RelatedFrameChangedSlot);
    QObject::connect(this->ui->comboBox_MappointList, &QComboBox::currentTextChanged, this, &MainWindow::MappointChangedSlot);
    QObject::connect(this->ui->comboBox_FrameList, &QComboBox::currentTextChanged, this, &MainWindow::FrameChangedSlot);
	
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::InitSystem(std::string path)
{
}

void MainWindow::SaveSystem(std::string path)
{
}

void MainWindow::LoadConfigSlot()
{
}

void MainWindow::SaveDataSlot()
{
}

void MainWindow::SaveConfigSlot()
{
}

void MainWindow::ComputeOnceSlot()
{
}

void MainWindow::RelatedFrameChangedSlot(QString index)
{
    
}

void MainWindow::FrameChangedSlot(QString index)
{
    bool ok;
    int i = index.toInt(&ok);
    if (ok)
    {
        this->ui->textEdit_FrameInfo->clear();

    }
    auto frame = this->GlobalMap->getFrameObject(i);
    
    std::stringstream str;
    str << frame << std::endl;
    std::string info = str.str();
    this->ui->textEdit_FrameInfo->setText(QString::fromStdString(info));
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

