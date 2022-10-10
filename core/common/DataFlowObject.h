#pragma once
#include "MVSObject.h"
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief this is the base class for all data storage types.
 * 
 */
class DataFlowObject : public MVSObject
{
public:
/**
 * @brief Construct a new Data Flow Object object
 * 
 */
    DataFlowObject();

    /**
     * @brief Destroy the Data Flow Object object
     * 
     */
    virtual ~DataFlowObject() {}

public:
/**
 * @brief the shared pointer type of DataFlowObject
 * 
 */
    using Ptr=std::shared_ptr<DataFlowObject>;

    /**
     * @brief create shared pointer of DataFlowObject,
     * do NOT use this method directly, use the Create() method in derived class.
     * 
     * @return DataFlowObject::Ptr 
     */
    //static DataFlowObject::Ptr Create();

    static void cvMat2Sophus(const cv::Mat1d& Mat, Sophus::SE3d& Pose);
    static void Sophus2cvMat(const Sophus::SE3d& pose, cv::Mat1d& Mat);
    static cv::Mat1d Rt2T(const cv::Mat1d& R, const cv::Mat1d& t);
    static std::tuple<cv::Mat1d, cv::Mat1d> T2Rt(const cv::Mat1d& T);
    static cv::Mat1d TK2F(const cv::Mat1d& K2, const cv::Mat1d& K1, const cv::Mat1d& T);

private:
    /* data */
};