#include "DataFlowObject.h"
#include <opencv2/core/eigen.hpp>

DataFlowObject::DataFlowObject()
{
    
}

//DataFlowObject::Ptr DataFlowObject::Create()
//{
//    return std::make_shared<DataFlowObject>();
//}

void DataFlowObject::cvMat2Sophus(const cv::Mat1d& Mat, Sophus::SE3d& Pose)
{
    Eigen::Matrix4d tmp;
    cv::cv2eigen(Mat, tmp);
    Pose = Sophus::SE3d(tmp);
}

void DataFlowObject::Sophus2cvMat(const Sophus::SE3d& pose, cv::Mat1d& Mat)
{
    Eigen::Matrix4d tmp = pose.matrix();
    cv::eigen2cv(tmp, Mat);
}

cv::Mat1d DataFlowObject::Rt2T(const cv::Mat1d& R, const cv::Mat1d& t)
{
    cv::Mat1d T = cv::Mat1d::eye(4, 4);
	R.copyTo(T(cv::Rect(0, 0, 3, 3)));
	t.copyTo(T(cv::Rect(3, 0, 1, 3)));

    return T;
}

std::tuple<cv::Mat1d, cv::Mat1d> DataFlowObject::T2Rt(const cv::Mat1d& T)
{
    cv::Mat1d R, t;
    R = T.rowRange(0, 3).colRange(0, 3).clone();
    t = T.rowRange(0, 3).colRange(3, 4).clone();
    return { R,t };
}