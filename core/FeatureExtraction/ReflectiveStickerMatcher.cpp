#include "ReflectiveStickerMatcher.h"
#include <opencv2/features2d.hpp>

ReflectiveStickerMatcher::Ptr ReflectiveStickerMatcher::Create(GlobalMapObject::Ptr GlobalMap)
{
    return std::make_shared<ReflectiveStickerMatcher>(GlobalMap);
}

ReflectiveStickerMatcher::ReflectiveStickerMatcher(GlobalMapObject::Ptr GlobalMap)
    :VocTreeMatcher(GlobalMap),
	CurrentFrameIndex(0),
	TotalFrameNumber(0)
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
    auto result = VocTreeMatcher::save(fs);
    fs["total-frame-number"] = this->TotalFrameNumber;
    return result;
}

bool ReflectiveStickerMatcher::load(JsonNode& fs)
{
    auto result = VocTreeMatcher::load(fs);
    this->TotalFrameNumber = fs.at("total-frame-number");
    this->m_isInit = result;
    this->CurrentFrameIndex = 0;
    return result;
}

bool ReflectiveStickerMatcher::Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
    if (!this->m_isInit)
    {
        emit this->Error(std::string("workflow is not initialized\n"));
        return false;
    }
    
    std::set<int> temp;
    frame->getAllRelatedFrames(temp);
    if (temp.size() == 0)
        return false;

	//找到位姿已知的关联帧，即双目产生帧
    std::set<int> RelatedFramesID;
    std::set<FrameObject::RelatedFrameInfo::Ptr> FramePtrWithKnownPos; //已知位姿的帧(双目产生)
    std::set<FrameObject::RelatedFrameInfo::Ptr> FranePtrWithoutPos; //未知位姿的帧(运动产生)

    if (this->CurrentFrameIndex != 0)
    {
        auto RelatedPtr = FrameObject::RelatedFrameInfo::Create(LastFrame);
        frame->addRelatedFrame(RelatedPtr);
    }
	
  //  if (this->CurrentFrameIndex == this->TotalFrameNumber)
  //  {
		//auto RelatedPtr = FrameObject::RelatedFrameInfo::Create(FirstFrame);
		//frame->addRelatedFrame(RelatedPtr);
  //  }
		
	
    frame->getAllRelatedFrames(RelatedFramesID);
    for (auto& item : RelatedFramesID)
    {
        auto RelatedPtr = frame->getRelatedFrame(item);
        if (RelatedPtr->getRelatedFrame()->MapID==frame->MapID) // 根据地图ID来判断是否是双目产生的
            //FranePtrWithoutPos.insert(RelatedPtr);
            FramePtrWithKnownPos.insert(RelatedPtr); //HACK:
        else 
            FranePtrWithoutPos.insert(RelatedPtr);
		
    }
	
	
    for (auto& RelatedPtr : FramePtrWithKnownPos)
    {
        cv::Mat CameraMat1, CameraMat2;
        CameraMat1 = std::static_pointer_cast<PinholeFrameObject>(frame)->CameraMatrix;
        CameraMat2 = std::static_pointer_cast<PinholeFrameObject>(RelatedPtr->getRelatedFrame())->CameraMatrix;

        cv::Mat1d T;
        DataFlowObject::Sophus2cvMat(*RelatedPtr->Pose, T);
        cv::Mat1d F = DataFlowObject::TK2F(CameraMat2, CameraMat1, T);	

          //test
      std::vector<cv::Point2f> tmp;
      for (auto& item : frame->KeyPoints)
      {
          if (item.class_id != MarkerPointType)
              continue;
          tmp.push_back(item.pt);
      }
      cv::Mat lines;
      cv::computeCorrespondEpilines(tmp, 1, F, lines);
      //draw lines
      cv::Mat img1 = frame->RGBMat.clone();
      cv::Mat img2 = RelatedPtr->getRelatedFrame()->RGBMat.clone();
      for (int i = 0; i < lines.rows; i++)
      {
          float a = lines.at<float>(i, 0);
          float b = lines.at<float>(i, 1);
          float c = lines.at<float>(i, 2);
          cv::Point pt1, pt2;
          pt1.x = 0;
          pt1.y = a * pt1.x + c;
          pt1.y /= -b;
      	
          pt2.x = 2000;
          pt2.y = a * pt2.x + c;
          pt2.y /= -b;
      	
          cv::line(img2, pt1, pt2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
      }

		
        this->ComputeWithF(frame->KeyPoints, RelatedPtr->getRelatedFrame()->KeyPoints, F, RelatedPtr->KeyPointMatch);
        cv::Mat look;
        cv::drawMatches(frame->RGBMat, frame->KeyPoints, RelatedPtr->getRelatedFrame()->RGBMat,
            RelatedPtr->getRelatedFrame()->KeyPoints, RelatedPtr->KeyPointMatch, look);
    }

    bool matched = false;
    for (auto& RelatedPtr : FranePtrWithoutPos)
    {
        cv::Mat CameraMat1, CameraMat2;
        CameraMat1 = std::static_pointer_cast<PinholeFrameObject>(frame)->CameraMatrix;
        CameraMat2 = std::static_pointer_cast<PinholeFrameObject>(RelatedPtr->getRelatedFrame())->CameraMatrix;
        cv::Mat1d F;
        this->MatchAndFindF(frame, RelatedPtr->getRelatedFrame(), F);

        //test
      std::vector<cv::Point2f> tmp;
      for (auto& item : frame->KeyPoints)
      {
          if (item.class_id != MarkerPointType)
              continue;
          tmp.push_back(item.pt);
      }
      cv::Mat lines;
      cv::computeCorrespondEpilines(tmp, 1, F, lines);
      //draw lines
      cv::Mat img1 = frame->RGBMat.clone();
      cv::Mat img2 = RelatedPtr->getRelatedFrame()->RGBMat.clone();
      for (int i = 0; i < lines.rows; i++)
      {
          float a = lines.at<float>(i, 0);
          float b = lines.at<float>(i, 1);
          float c = lines.at<float>(i, 2);
          cv::Point pt1, pt2;
          pt1.x = 0;
          pt1.y = a * pt1.x + c;
          pt1.y /= -b;
      	
          pt2.x = 2000;
          pt2.y = a * pt2.x + c;
          pt2.y /= -b;
      	
          cv::line(img2, pt1, pt2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
      }

		this->ComputeWithF(frame->KeyPoints, RelatedPtr->getRelatedFrame()->KeyPoints, F, RelatedPtr->KeyPointMatch);
        cv::Mat look;
        cv::drawMatches(frame->RGBMat, frame->KeyPoints, RelatedPtr->getRelatedFrame()->RGBMat,
            RelatedPtr->getRelatedFrame()->KeyPoints, RelatedPtr->KeyPointMatch, look);
        
        if (RelatedPtr->KeyPointMatch.size() > 6)
            matched = true;
    }
	

	
    if (matched) //解决匹配错误的情况
        this->LastFrame = frame;
		
    if (this->CurrentFrameIndex == 0)
    {
		this->FirstFrame = frame;
		this->LastFrame = frame;
    }
	
	this->CurrentFrameIndex++;

	return true;
}


void ReflectiveStickerMatcher::ComputeWithF(const std::vector<cv::KeyPoint>& KeyPoints1, const std::vector<cv::KeyPoint>& KeyPoints2, const cv::Mat1d& F, std::vector<cv::DMatch>& Matches)
{
    std::vector<cv::Point2f> tmp1;
    for (auto& item : KeyPoints1)
    {
        if (item.class_id != MarkerPointType)
            continue;
        tmp1.push_back(item.pt);
    }

    std::vector<cv::Point2f> tmp2;
    for (auto& item : KeyPoints2)
    {
        if (item.class_id != MarkerPointType)
            continue;
        tmp2.push_back(item.pt);
    }

    cv::Mat Line1, Line2;
    cv::computeCorrespondEpilines(tmp1, 1, F, Line1);
    cv::computeCorrespondEpilines(tmp2, 2, F, Line2);
	
    int index = 0;
	int count1 = -1; //因为在continue之前++了，所以是-1
    int count2 = -1; //解决特征点的编号问题，非常讨厌的补丁式编程，就这样吧
    std::vector<cv::DMatch> tmpMatches;
    for (auto& Point1 : KeyPoints1)
    {
        int index2 = 0;
        count2 = -1;
        count1++;
        if (Point1.class_id != MarkerPointType)
        {
            continue;
        }

		
        for (auto& Points2 : KeyPoints2)
        {
			count2++;
            if (Points2.class_id != MarkerPointType)
            {
				continue;
            }

			double a = Line1.at<float>(index, 0);
            double b = Line1.at<float>(index, 1);
            double c = Line1.at<float>(index, 2);
			
            double distance = abs(a * Points2.pt.x + b * Points2.pt.y + c) / sqrt(a * a + b * b);

            double PointDistance = sqrt(pow(Point1.pt.x - Points2.pt.x, 2) + pow(Point1.pt.y - Points2.pt.y, 2));
            if (distance < 50)
            {
                tmpMatches.emplace_back(count1, count2,PointDistance);
            }
            index2++;
        }
        index++;
    }
	
    if (tmpMatches.empty())
        return;

    double AvgDistance = 0;
    std::sort(tmpMatches.begin(), tmpMatches.end(), [](cv::DMatch A, cv::DMatch B) {
        return A.distance > B.distance;
        });

    AvgDistance = tmpMatches.at(tmpMatches.size() / 2).distance;

    std::map<int, std::list<cv::DMatch>> matchCount;
    std::vector<cv::DMatch> tmpMatches2;
	for (auto& item : tmpMatches)
	{
		if (item.distance < AvgDistance * 3 && item.distance > AvgDistance / 4)
		{
            tmpMatches2.push_back(item);
            matchCount[item.queryIdx].push_back(item);
		}
	}
	
    std::map<int, std::list<cv::DMatch>> matchCount2;
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
            //MatchesList.push_back(avg);
			matchCount2[avg.trainIdx].push_back(avg);
		}
        else
        {
            //MatchesList.push_back(item.second.front());
            matchCount2[item.second.front().trainIdx].push_back(item.second.front());
        }
	}

	for (auto& item : matchCount2)
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

bool ReflectiveStickerMatcher::MatchAndFindF(FrameObject::Ptr frame1, FrameObject::Ptr frame2, cv::Mat1d& F)
{
    std::vector<cv::DMatch> matches;
    auto result = VocTreeMatcher::MatchKeyPoints(frame1, frame2, matches);
    if (!result) return false;
	
    cv::Mat look;
    cv::drawMatches(frame1->RGBMat, frame1->KeyPoints, frame2->RGBMat,
        frame2->KeyPoints, matches, look);

    std::vector<cv::Point2f> MatchedPoint1, MatchedPoint2;
    MatchedPoint1.reserve(matches.size());
    MatchedPoint2.reserve(matches.size());
	for (auto& item : matches)
	{
		MatchedPoint1.push_back(frame1->KeyPoints[item.queryIdx].pt);
		MatchedPoint2.push_back(frame2->KeyPoints[item.trainIdx].pt);
	}
	
    auto F_Matrix = cv::findFundamentalMat(MatchedPoint1, MatchedPoint2, cv::FM_RANSAC, 3, 0.9); //使用ransac方法+默认参数
	
	if (F_Matrix.empty())
		return false;

	F = F_Matrix;
	return true;
}
