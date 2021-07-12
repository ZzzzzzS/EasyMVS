#include "GreenIntegralImage.h"
#include <iostream>
#include <qdebug.h>


GreenIntegralImage::GreenIntegralImage(cv::Mat OfflineMat)
{
	this->UpdateGreenIntegralImage(OfflineMat);
}
void GreenIntegralImage::UpdateGreenIntegralImage(cv::Mat OfflineMat)
{
	OfflineMat.copyTo(this->OfflineMat);
	this->CalculateIntegral_P();
	this->CalculateIntegral_Q();
}

GreenIntegralImage::~GreenIntegralImage()
{
}

cv::Mat GreenIntegralImage::GetIntegralMap_Q()
{
	return this->IntegralMap_Q;
}

cv::Mat GreenIntegralImage::GetIntegralMap_P()
{
	return this->IntegralMap_P;
}

float GreenIntegralImage::Sigma(std::vector<cv::Point> Points, int chennel)
{
	cv::Point LastPoint;

	float Sigma = 0;

	auto ImagePtr_Q = this->IntegralMap_Q.ptr<float>(0) + chennel;
	auto ImagePtr_P = this->IntegralMap_P.ptr<float>(0) + chennel; //在最开始的指针就计算好通道偏移量，就不用在后面每次计算了
	auto cols = this->IntegralMap_P.cols;

	LastPoint = Points[0];
	for (auto i : Points)
	{
		int DeltaX = i.x - LastPoint.x; //计算deltaX，deltaY
		int DeltaY = i.y - LastPoint.y;

		Sigma += DeltaX * ImagePtr_P[(i.x + (i.y - 1)*cols) * 3] + DeltaY * ImagePtr_Q[(i.x + (i.y - 1)*cols) * 3];

		if (Sigma)
		{
			__nop();
		}

		LastPoint = i;
		/*if (LastPoint.x==i.x && LastPoint.y<i.y) //向下搜索
		{

		}
		else if (LastPoint.x == i.x && LastPoint.y > i.y) //向上搜索
		{

		}
		else if (LastPoint.y = i.y&&LastPoint.x < i.x)//向右搜索
		{

		}
		else if (LastPoint.y == i.y&&LastPoint.x > i.x)//向左搜索
		{

		}
		else
		{
			qDebug() << "非法的位置";
			qDebug() << "上次位置：x=" << LastPoint.x << "y=" << LastPoint.y << "新位置：x=" << i.x << "y=" << i.y;
		}*/
	}
	return Sigma;
}

cv::Vec3f GreenIntegralImage::Sigma3(std::vector<cv::Point> Points)
{
	cv::Vec3f ArrayOfSigma;
	ArrayOfSigma[0] = this->Sigma(Points, 0);
	ArrayOfSigma[1] = this->Sigma(Points, 1);
	ArrayOfSigma[2] = this->Sigma(Points, 2);

	return ArrayOfSigma;
}

void GreenIntegralImage::CalculateIntegral_Q()
{
	this->IntegralMap_Q = cv::Mat::zeros(this->OfflineMat.size(), CV_32FC3);

	for (size_t i = 0; i < OfflineMat.rows; i++)
	{
		float C1 = 0;
		float C2 = 0;
		float C3 = 0;
		auto P = this->OfflineMat.ptr<uchar>(i);
		auto Q = this->IntegralMap_Q.ptr<float>(i);

		for (size_t j = 0; j < OfflineMat.cols; j++)
		{
			C1 += 0.5*P[j * 3];
			C2 += 0.5*P[j * 3 + 1];
			C3 += 0.5*P[j * 3 + 2];

			Q[j * 3] = C1;
			Q[j * 3 + 1] = C2;
			Q[j * 3 + 2] = C3;
		}
	}
}

void GreenIntegralImage::CalculateIntegral_P()
{
	this->IntegralMap_P = cv::Mat::zeros(this->OfflineMat.size(), CV_32FC3);

	auto PointerO = this->OfflineMat.ptr<uchar>(0);
	auto PointerP = this->IntegralMap_P.ptr<float>(0);

	for (size_t i = 0; i < this->OfflineMat.cols; i++)
	{
		float C1 = 0;
		float C2 = 0;
		float C3 = 0;

		for (size_t j = 0; j < this->OfflineMat.rows; j++)
		{
			C1 += -0.5*PointerO[(i + j * this->OfflineMat.cols) * 3];
			C2 += -0.5*PointerO[(i + j * this->OfflineMat.cols) * 3 + 1];
			C3 += -0.5*PointerO[(i + j * this->OfflineMat.cols) * 3 + 2];

			PointerP[(i + j * this->OfflineMat.cols) * 3] = C1;
			PointerP[(i + j * this->OfflineMat.cols) * 3 + 1] = C2;
			PointerP[(i + j * this->OfflineMat.cols) * 3 + 2] = C3;
		}
	}
}

