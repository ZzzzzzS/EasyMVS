#include "DenseReconstruction.h"
#include <QtWidgets/QApplication>

#include "GreenIntegralImage.h"

void test()
{
	//cv::Mat a = cv::Mat::eye(100, 100, CV_8UC3);
	//GreenIntegralImage b(a);

	cv::Mat InMat = cv::imread("C:/Users/ZhouZishun/Desktop/Screenshot 2021-07-05 140706.png");
	GreenIntegralImage Acc(InMat);

	float sum = 0;

	for (size_t i = 50; i <100; i++)
	{
		auto Ptr = InMat.ptr<uchar>(i);
		for (size_t j = 25; j < 75; j++)
		{
			sum += Ptr[j * 3];
		}
	}

	std::vector<cv::Point> Edge;

	for (size_t i = 50; i <= 100; i++)
	{
		cv::Point in;
		in.y = 25;
		in.x = i;
		Edge.push_back(in);
	}

	for (size_t i = 25; i <= 75; i++)
	{
		cv::Point in;
		in.y = i;
		in.x = 100;
		Edge.push_back(in);
	}

	for (size_t i =	100; i >= 50; i--)
	{
		cv::Point in;
		in.y = 75;
		in.x = i;
		Edge.push_back(in);
	}

	for (size_t i = 75; i >= 25; i--)
	{
		cv::Point in;
		in.y = i;
		in.x = 50;
		Edge.push_back(in);
	}

	float sum2 = Acc.Sigma(Edge, 0);
	__nop();
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DenseReconstruction w;
    w.show();
	test();
    return a.exec();
}
