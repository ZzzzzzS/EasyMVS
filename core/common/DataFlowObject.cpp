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

cv::Mat1d DataFlowObject::Rt2T(cv::Mat1d& R, cv::Mat1d& t)
{
    cv::Mat1d T = cv::Mat1d::eye(4, 4);

    T = cv::Mat::zeros(4, 4, CV_64FC1);
    double* RPtr = R.ptr<double>(0);
    double* tPtr = t.ptr<double>(0);
    double* TPtr = T.ptr<double>(0);

    TPtr[0] = RPtr[0]; TPtr[1] = RPtr[1]; TPtr[2] = RPtr[2];  TPtr[3] = tPtr[0];
    TPtr[4] = RPtr[3]; TPtr[5] = RPtr[4]; TPtr[6] = RPtr[5];  TPtr[7] = tPtr[1];
    TPtr[8] = RPtr[6]; TPtr[9] = RPtr[7]; TPtr[10] = RPtr[8]; TPtr[11] = tPtr[2];
    TPtr[12] = 0;      TPtr[13] = 0;      TPtr[14] = 0;       TPtr[15] = 1;

    return T;
}

std::tuple<cv::Mat1d, cv::Mat1d> DataFlowObject::T2Rt(const cv::Mat1d& T)
{
    cv::Mat1d R, t;
    R = T.rowRange(0, 2).colRange(0, 2).clone();
    t = T.rowRange(0, 2).colRange(3, 3).clone();
    return { R,t };
}