#include "ReflectiveStickerExtractor.h"
#include <nlohmann/json.hpp>


void MaxFilter(cv::Mat& src_image, cv::Mat& dst_image, int k_size);

ReflectiveStickerExtractor::ReflectiveStickerExtractor()
{
    this->m_isInit=true;
}

ReflectiveStickerExtractor::~ReflectiveStickerExtractor()
{
    
}

bool ReflectiveStickerExtractor::Compute(FrameObject::Ptr frame)
{
    if (frame->RGBMat.empty())
    {
        std::cout<<this->type_name() << ": RGBMat is empty" << std::endl;
        return false;
    }
    CVFeatureExtractor::Compute(frame); //先进行特征点识别
    this->ComputeReflectiveSticker(frame->RGBMat,frame->KeyPoints);
    return frame->KeyPoints.empty() ? false : true;
}

bool ReflectiveStickerExtractor::load(JsonNode& fs)
{
    bool result = CVFeatureExtractor::load(fs);
    //this->m_isInit=true;
    return result;
}

bool ReflectiveStickerExtractor::save(JsonNode& fs)
{
	return CVFeatureExtractor::save(fs);
    //return true;
}

std::string ReflectiveStickerExtractor::type_name()
{
    return std::string("reflective-sticker-extractor");
}

bool ReflectiveStickerExtractor::clear()
{
    this->m_isInit=false;
    return true;
}

void ReflectiveStickerExtractor::ComputeReflectiveSticker(cv::Mat& img,std::vector<cv::KeyPoint>& KeyPoints)
{
    cv::Mat gray;
    cv::cvtColor(img,gray,cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    double MeanValue=cv::mean(gray)[0];
    cv::threshold(gray,binary,MeanValue,255,cv::THRESH_BINARY_INV);
    //cv::GaussianBlur(binary, binary, cv::Size(5, 5), 5);
    cv::Mat bin2(binary.size(), CV_8UC1);
	MaxFilter(binary, bin2, 5);

    cv::Mat1i labels, stats;
    cv::Mat1d centroids;
    int nLabels = cv::connectedComponentsWithStats(bin2, labels, stats, centroids, 8, CV_32S);

	cv::Mat1i labels2, stats2;
	cv::Mat1d centroids2;
    cv::Mat bin3(bin2.size(), CV_8UC1);
    bin2.forEach<uchar>([&bin3](uchar& p, const int* position) {
        bin3.at<uchar>(position[0], position[1]) = (p != 0) ? 0 : 255;    
        });
    int nLabels2 = cv::connectedComponentsWithStats(bin3, labels2, stats2, centroids2, 8, CV_32S);


    std::vector<int> PotentialPoints1;
    for (size_t i = 1; i < stats.rows; i++)
    {
        if (stats(i, cv::CC_STAT_AREA) < 5000 || stats(i, cv::CC_STAT_AREA) > 100000)
            continue;
		
        int width = stats(i, cv::CC_STAT_WIDTH);
		int height = stats(i, cv::CC_STAT_HEIGHT);
        if ((double)width / (double)height > 1.8 || (double)width / (double)height < 0.5)
            continue;
		
		PotentialPoints1.push_back(i);
    }
	
    

    std::vector<int> PotentialPoints2;
    for (size_t i = 0; i < stats2.rows; i++)
    {
        if (stats2(i, cv::CC_STAT_AREA) < 500 || stats2(i, cv::CC_STAT_AREA) > 10000)
            continue;

        int width = stats2(i, cv::CC_STAT_WIDTH);
        int height = stats2(i, cv::CC_STAT_HEIGHT);
        if ((double)width / (double)height > 1.8 || (double)width / (double)height < 0.5)
            continue;

        PotentialPoints2.push_back(i);
    }

    cv::Mat show2 = labels.clone();
    for (auto& item : PotentialPoints2)
    {
        cv::circle(img, cv::Point(centroids2(item, 0), centroids2(item, 1)), 10, cv::Scalar(128), 3);
        cv::circle(show2, cv::Point(centroids2(item, 0), centroids2(item, 1)), 10, cv::Scalar(128), 3);
    }

    //std::vector<int> PotentialPoints3;
    //for (auto& item2 : PotentialPoints2)
    //{
    //    double MinDistance = DBL_MAX;
    //    int index = -1;
    //    for (auto& item1 : PotentialPoints1)
    //    {
    //        double distance = std::pow(centroids(item1, 0) - centroids2(item2, 0), 2) +
    //            std::pow(centroids(item1, 1) - centroids2(item2, 1), 2);

    //        if (distance < MinDistance)
    //        {
    //            MinDistance = distance;
    //            index = item2;
    //        }
    //            
    //    }

    //    //std::cout << item2 << "mindist" << MinDistance<<std::endl;
    //    if (MinDistance < 100)
    //        PotentialPoints3.push_back(item2);
    //}

    //cv::Mat show = gray.clone();
    //for (auto& item : PotentialPoints3)
    //{
    //    cv::circle(show, cv::Point(centroids2(item, 0), centroids2(item, 1)), 10, cv::Scalar(128), 3);
    //}

    KeyPoints.reserve(PotentialPoints2.size()+KeyPoints.size());
    for (auto& item : PotentialPoints2)
    {
        KeyPoints.emplace_back(centroids2(item, 0), centroids2(item, 1), stats2(item, cv::CC_STAT_AREA), -1, 0, 0, MarkerPointType);
    }
}

ReflectiveStickerExtractor::Ptr ReflectiveStickerExtractor::Create()
{
    return std::make_shared<ReflectiveStickerExtractor>();
}


/* 自定义最大值滤波，邻域内最大，卷积核大小默认3 */
//https://www.cnblogs.com/cralor/p/14054254.htmls
//网上的代码还是不太行啊，有一堆bug，早知道自己写了
void MaxFilter(cv::Mat& src_image, cv::Mat& dst_image, int k_size) {
    int max_rows = src_image.rows;          // 行像素数
    int max_cols = src_image.cols;          // 列像素数
    int channels = src_image.channels();    // 图片是几通道的

    int p = (k_size - 1) / 2;               // -(k−1)/2 ~ (k−1)/2

    int maxt = 0;                           // 初始最小值;

    // 对每个像素点进行处理
    for (int row = 0; row < max_rows; ++row) {
        for (int col = 0; col < max_cols; ++col) {
            // 1通道，灰度值
            maxt = 0;
            if (channels == 1) {
                // 以当前像素点为中心的k*k的矩阵中，取最大值
                for (int i = row - p; i <= row + p; ++i)
                    for (int j = col - p; j <= col + p; ++j)
                        if (i >= 0 && i < max_rows && j >= 0 && j < max_cols)
                            if (src_image.at<uchar>(i, j) > maxt)
                                maxt = src_image.at<uchar>(i, j);

                dst_image.at<uchar>(row, col) = maxt;   // 像素值赋最大值    
            }// if

        }// else if
     }// for 
}