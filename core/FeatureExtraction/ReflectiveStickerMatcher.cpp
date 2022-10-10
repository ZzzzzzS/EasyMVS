#include "ReflectiveStickerMatcher.h"
#include <opencv2/features2d.hpp>

ReflectiveStickerMatcher::Ptr ReflectiveStickerMatcher::Create(GlobalMapObject::Ptr GlobalMap)
{
    return std::make_shared<ReflectiveStickerMatcher>(GlobalMap);
}

ReflectiveStickerMatcher::ReflectiveStickerMatcher(GlobalMapObject::Ptr GlobalMap)
    :FeatureMatcher(GlobalMap)
{
}

ReflectiveStickerMatcher::~ReflectiveStickerMatcher()
{
    
}

std::string ReflectiveStickerMatcher::type_name()
{
    return std::string("workflow-reflective-sticker-matcher");
}

bool ReflectiveStickerMatcher::clear()
{
    this->m_isInit=false;
    return true;
}

bool ReflectiveStickerMatcher::save(JsonNode& fs)
{
    return true;
}

bool ReflectiveStickerMatcher::load(JsonNode& fs)
{
    this->m_isInit=true;
    return true;
}

bool ReflectiveStickerMatcher::Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
    if (!this->m_isInit)
    {
        emit this->Error(std::string("workflow is not initialized\n"));
        return false;
    }
    
	//找到位姿已知的关联帧，即双目产生帧
    std::set<int> RelatedFramesID;
    std::set<FrameObject::RelatedFrameInfo::Ptr> FramePtrWithKnownPos;
    frame->getAllRelatedFrames(RelatedFramesID);
    for (auto& item : RelatedFramesID)
    {
        auto RelatedPtr = frame->getRelatedFrame(item);
        if (RelatedPtr->getRelatedFrame()->isGlobalPoseKnown())
            FramePtrWithKnownPos.insert(RelatedPtr);
    }
	
	
    for (auto& RelatedPtr : FramePtrWithKnownPos)
    {
        cv::Mat CameraMat1, CameraMat2;
        CameraMat1 = std::static_pointer_cast<PinholeFrameObject>(frame)->CameraMatrix;
        CameraMat2 = std::static_pointer_cast<PinholeFrameObject>(RelatedPtr->getRelatedFrame())->CameraMatrix;

        cv::Mat1d T;
        DataFlowObject::Sophus2cvMat(*RelatedPtr->Pose, T);
		//
  //      cv::Mat1d K1_inv = cv::Mat1d::zeros(4, 3);
  //      K1_inv(3, 2) = 1;
  //      cv::Mat1d K1_invtmp = CameraMat1.inv();
  //      K1_invtmp.copyTo(K1_inv.colRange(0, 3).rowRange(0, 3));

  //      cv::Mat1d K2 = cv::Mat1d::zeros(3, 4);
  //      CameraMat2.copyTo(K2.colRange(0, 3).rowRange(0, 3));

  //      //cv::Mat1d H_Mat = K2 * T.inv() * K1_inv;
  //      cv::Mat1d H_Mat = CameraMat2 * T(cv::Rect(0, 0, 3, 3)) * (CameraMat1.inv());
  //      H_Mat = H_Mat / H_Mat(2, 2);

		////test
  //      cv::Mat TransMat;
  //      cv::warpPerspective(frame->RGBMat, TransMat, H_Mat, frame->RGBMat.size() * 5);
  //      __nop();

  //      RelatedPtr->getRelatedFrame()->RGBMat.copyTo(TransMat(cv::Rect(0, 0, 2448, 2048)));
        /*cv::Mat R1, R2, P1, P2, Q;
        auto [R, t] = DataFlowObject::T2Rt(T);
        cv::stereoRectify(CameraMat1, std::static_pointer_cast<PinholeFrameObject>(frame)->DistCoeff,
            CameraMat2, std::static_pointer_cast<PinholeFrameObject>(RelatedPtr->getRelatedFrame())->DistCoeff,
            frame->RGBMat.size(), R, t, R1, R2, P1, P2, Q, 0, -1);
		
        cv::Mat map1x, map1y, map2x, map2y;
        cv::initUndistortRectifyMap(CameraMat1, std::static_pointer_cast<PinholeFrameObject>(frame)->DistCoeff, R1, P1, frame->RGBMat.size(), CV_32FC2, map1x, map1y);
        cv::initUndistortRectifyMap(CameraMat2, std::static_pointer_cast<PinholeFrameObject>(RelatedPtr->getRelatedFrame())->DistCoeff, R2, P2, frame->RGBMat.size(), CV_32FC2, map2x, map2y);

        cv::Mat Pinhole1Rectified, Pinhole2Rectified;
        cv::remap(frame->RGBMat, Pinhole1Rectified, map1x, map1y, cv::INTER_LINEAR);
        cv::remap(RelatedPtr->getRelatedFrame()->RGBMat, Pinhole2Rectified, map2x, map2y, cv::INTER_LINEAR);
		*/
		
        cv::Mat1d F = DataFlowObject::TK2F(CameraMat2, CameraMat1, T);
		
		//test
  //      std::vector<cv::Point2f> tmp;
  //      for (auto& item : frame->KeyPoints)
  //      {
  //          tmp.push_back(item.pt);
  //      }
  //      cv::Mat lines;
		//cv::computeCorrespondEpilines(tmp, 2, F, lines);
		////draw lines
		//cv::Mat img1 = frame->RGBMat.clone();
		//cv::Mat img2 = RelatedPtr->getRelatedFrame()->RGBMat.clone();
  //      for (int i = 0; i < lines.rows; i++)
  //      {
  //          float a = lines.at<float>(i, 0);
  //          float b = lines.at<float>(i, 1);
  //          float c = lines.at<float>(i, 2);
  //          cv::Point pt1, pt2;
  //          pt1.x = 0;
  //          pt1.y = a * pt1.x + c;
  //          pt1.y /= -b;
		//	
  //          pt2.x = 2000;
  //          pt2.y = a * pt2.x + c;
  //          pt2.y /= -b;
		//	
  //          cv::line(img2, pt1, pt2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
  //      }
        this->ComputeWithF(frame->KeyPoints, RelatedPtr->getRelatedFrame()->KeyPoints, F, RelatedPtr->KeyPointMatch);
        cv::Mat look;
        cv::drawMatches(frame->RGBMat, frame->KeyPoints, RelatedPtr->getRelatedFrame()->RGBMat,
            RelatedPtr->getRelatedFrame()->KeyPoints, RelatedPtr->KeyPointMatch, look);
    }

	return true;
}

bool ReflectiveStickerMatcher::MatchWithKnownH(const std::vector<cv::KeyPoint>& Points1, const std::vector<cv::KeyPoint>& Point2, cv::InputArray H, std::vector<cv::DMatch>& Matches)
{
	//计算反射贴纸的特征点在另一帧图像中的投影
	
    return false;
}

void ReflectiveStickerMatcher::ComputeWithF(const std::vector<cv::KeyPoint>& Points1, const std::vector<cv::KeyPoint>& Points2, const cv::Mat1d& F, std::vector<cv::DMatch>& Matches)
{
    std::vector<cv::Point2f> tmp1;
    for (auto& item : Points1)
    {
        tmp1.push_back(item.pt);
    }

    std::vector<cv::Point2f> tmp2;
    for (auto& item : Points2)
    {
        tmp2.push_back(item.pt);
    }

    cv::Mat Line1, Line2;
    cv::computeCorrespondEpilines(tmp1, 2, F, Line1);
    cv::computeCorrespondEpilines(tmp2, 1, F, Line2);
	
    int index = 0;
    std::vector<cv::DMatch> tmpMatches;
    for (auto& Point1 : tmp1)
    {
        int index2 = 0;
        for (auto& Points2 : tmp2)
        {
			double a = Line1.at<float>(index, 0);
            double b = Line1.at<float>(index, 1);
            double c = Line1.at<float>(index, 2);
			
            double distance = abs(a * Points2.x + b * Points2.y + c) / sqrt(a * a + b * b);

            double PointDistance = pow(Point1.x - Points2.x, 2) + pow(Point1.y - Points2.y, 2);
            if (distance < 10)
            {
                tmpMatches.emplace_back(index, index2,PointDistance);
            }
            index2++;
        }
        index++;
    }
	
    double AvgDistance = 0;
    std::sort(tmpMatches.begin(), tmpMatches.end(), [](cv::DMatch A, cv::DMatch B) {
        return A.distance > B.distance;
        });

    AvgDistance = tmpMatches.at(tmpMatches.size() / 2).distance;

    std::map<int, std::list<cv::DMatch>> matchCount;
    std::vector<cv::DMatch> tmpMatches2;
	for (auto& item : tmpMatches)
	{
		if (item.distance < AvgDistance * 2 && item.distance > AvgDistance / 2)
		{
            tmpMatches2.push_back(item);
            matchCount[item.queryIdx].push_back(item);
		}
	}
	
	for (auto& item : matchCount)
	{
		if (item.second.size() > 1)
		{
            double distance = DBL_MAX;
            cv::DMatch avg;
            for (auto& i : item.second)
            {
                double localdistance = abs(i.distance - AvgDistance);
                if (localdistance < distance)
                {
                    distance = localdistance;
                    avg = i;
                }
            }
            Matches.push_back(avg);
		}
        else
        {
            Matches.push_back(item.second.front());
        }
	}
}
