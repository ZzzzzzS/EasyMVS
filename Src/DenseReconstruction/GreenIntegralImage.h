#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class GreenIntegralImage
{
public:
	GreenIntegralImage(cv::Mat OfflineMat);  
	void UpdateGreenIntegralImage(cv::Mat OfflineMat); //更新需要格林加速的图
	~GreenIntegralImage();

	cv::Mat GetIntegralMap_Q(); //获得积分图
	cv::Mat GetIntegralMap_P();

	float Sigma(std::vector< cv::Point > Points, int chenne); //计算由Points围成的多边形的面积和
	cv::Vec3f Sigma3(std::vector< cv::Point > Points); //计算所有通道

private:
	void CalculateIntegral_Q();
	void CalculateIntegral_P();

	cv::Mat IntegralMap_Q; //积分图Q，沿X方向积分图
	cv::Mat IntegralMap_P; //积分图P，沿Y方向积分图
	cv::Mat OfflineMat; //离线输入的图
};

