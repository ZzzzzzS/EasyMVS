#pragma once
#include "GlobalMapObject.h"
#include "FrameObject.h"
#include "MapPointObject.h"
#include "CameraObject.h"
#include "CameraModule/Photographer.h"
#include "FeatureExtraction/FeatureExtractor.h"
#include "CameraModule/ImageReaderCamera.h"
#include "FeatureExtraction/ReflectiveStickerExtractor.h"
#include "FeatureExtraction/ReflectiveStickerMatcher.h"
#include "PoseReconstruction/PinholePoseReconstructor.h"
#include "PoseReconstruction/ReflectiveReconstructor.h"
#include "CameraModule/MemImgReaderCamera.h"

#include <QObject>
#include <QThread>

class EasyMVSWarpper : public QObject
{
public:
	Q_OBJECT

public:
/**
 * @brief Construct a new EasyMVSWarpper object
 * 
 */
	EasyMVSWarpper();

	/**
	 * @brief Construct a new EasyMVSWarpper object
	 * 
	 * @param LeftCameraMatrix 左相机内参
	 * @param LeftDistcoeff 左相机畸变参数
	 * @param RightCameraMatrix 右相机内参
	 * @param RightDistcoeff 右相机畸变参数
	 * @param TL2R 左相机到右相机的变换矩阵
	 */
	EasyMVSWarpper(cv::InputArray LeftCameraMatrix, cv::InputArray LeftDistcoeff,
		cv::InputArray RightCameraMatrix, cv::InputArray RightDistcoeff, cv::InputArray TL2R);
	
	/**
	 * @brief Destroy the EasyMVSWarpper object
	 * 
	 */
	~EasyMVSWarpper();

	/**
	 * @brief 加载重建参数
	 * 
	 * @param filename json文件名
	 * @return true 
	 * @return false 
	 */
	bool LoadParameters(std::string filename);

	/**
	 * @brief 保存重建参数
	 * @warning 该函数会丢失一些信息，如SIFT的参数
	 * 
	 * @param filename 保存文件名
	 * @return true 
	 * @return false 
	 */
	bool SaveParameters(std::string filename);

	/**
	 * @brief 保存重建帧姿态
	 * 
	 * @param filename 保存的文件名
	 * @return true 
	 * @return false 
	 */
	bool SaveData(std::string filename);

	/**
	 * @brief 加载用于重建的图像
	 * 
	 * @param Left 左相机图像
	 * @param Right 右相机图像
	 * @return true 加载成功
	 * @return false 加载失败，上一次重建未完成时无法加载新图像
	 */
	bool LoadImages(cv::InputArrayOfArrays Left, cv::InputArrayOfArrays Right);

public://slot
/**
 * @brief 开始重建
 * 
 */
	void Trigger();

signals:
/**
 * @brief 重建完成信号
 * 
 * @param Pose map<有效帧ID,帧位姿>
 * @details 帧编号从0开始，奇数帧为左相机产生的帧，偶数帧为右相机产生的帧。例如0帧是右相机产生的，1帧是左相机产生的。
 */
	void Finished(std::map<int,Sophus::SE3d> Pose);

	/**
	 * @brief 内部信号，请勿调用
	 * 
	 */
	void TriggerOnce();

private:
	CameraObject::Ptr CameraLeft;
	CameraObject::Ptr CameraRight;
	Photographer::Ptr Photographer1;
	FeatureExtractor::Ptr Extractor1;
	FeatureMatcher::Ptr Matcher1;
	PoseReconstructor::Ptr PoseReconstructor1;

	GlobalMapObject::Ptr GlobalMap;

	int FrameCount = 0;
private: //slot
	void FinishedOnce(WorkFlowObject::DataQueue queue);

};

