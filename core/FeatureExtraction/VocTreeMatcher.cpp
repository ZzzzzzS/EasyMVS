#include "VocTreeMatcher.h"
#include <opencv2/xfeatures2d.hpp>

VocTreeMatcher::Ptr VocTreeMatcher::Create(GlobalMapObject::Ptr GlobalMap)
{
	return std::make_shared<VocTreeMatcher>(GlobalMap);
}

VocTreeMatcher::VocTreeMatcher(GlobalMapObject::Ptr GlobalMap)
    :FeatureMatcher(GlobalMap),
	m_MatcherType(MatcherType_e::FLANNBASED),
	m_KeyPointType(KeyPointType_e::SIFT),
	bigRotation(false),
	bigScale(false),
	GMSthresholdFactor(6.0),
	MatchingNumber(5),
	LeastMatchingPoint(20)
{
}

VocTreeMatcher::~VocTreeMatcher()
{
}

bool VocTreeMatcher::Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
	try
	{
		auto map = (GlobalMap == nullptr) ? this->GlobalMap : GlobalMap;
		std::list<FrameObject::Ptr> related;
		map->addFrameObject(frame);

		auto result = this->MatchRelatedFrame(frame, related, map);
		if (!result)
			return false;

		bool hasMatched = false;
		for (auto& i : related)
		{
			std::vector<cv::DMatch> matches;
			auto matched = this->MatchKeyPoints(frame, i, matches);
			if (matched)
			{
				auto RelatedFrame = FrameObject::RelatedFrameInfo::Create(i, matches);
				frame->addRelatedFrame(RelatedFrame);
				hasMatched = true;
			}
		}
		return hasMatched;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
    return false;
}

bool VocTreeMatcher::MatchRelatedFrame(FrameObject::Ptr frame, std::list<FrameObject::Ptr>& related, GlobalMapObject::Ptr GlobalMap)
{
	auto map = (GlobalMap == nullptr) ? this->GlobalMap : GlobalMap;
	try
	{
		if (map->getFrameSize()==0)
		{
			return false;
		}
		else if (map->getFrameSize() == 1) //initial frame
		{
			switch (this->m_KeyPointType)
			{
			case VocTreeMatcher::KeyPointType_e::ORB:
			{
				//auto id1 = std::get<OrbDBPtr>(this->Voc_DB)->add(frame->KeyPointsDescriptors);
				auto id1 = this->m_Orbdb->add(frame->KeyPointsDescriptors);
				this->MatcherIndexMap.insert({ id1,frame });
				break;
			}
			case VocTreeMatcher::KeyPointType_e::SIFT:
			{
				//auto id2 = std::get<SiftDBPtr>(this->Voc_DB)->add(DBoW2::FSift128::fromMat32F(frame->KeyPointsDescriptors));
				auto id2=this->m_Siftdb->add(DBoW2::FSift128::fromMat32F(frame->KeyPointsDescriptors));
				this->MatcherIndexMap.insert({ id2,frame });
				break;
			}
			default:
				std::cout << this->type_name() << ": unknown matcher type" << std::endl;
				throw std::exception();
				break;
			}
			return true;
		}
		else
		{
			DBoW2::QueryResults ret;
			switch (this->m_KeyPointType)
			{
			case VocTreeMatcher::KeyPointType_e::ORB:
			{
				//std::get<OrbDBPtr>(this->Voc_DB)->query(frame->KeyPointsDescriptors, ret, this->MatchingNumber);
				this->m_Orbdb->query(frame->KeyPointsDescriptors, ret, this->MatchingNumber);
				//auto id1 = std::get<OrbDBPtr>(this->Voc_DB)->add(frame->KeyPointsDescriptors);
				auto id1=this->m_Orbdb->add(frame->KeyPointsDescriptors);
				this->MatcherIndexMap.insert({ id1,frame });
				break;
			}
			case VocTreeMatcher::KeyPointType_e::SIFT:
			{
				//std::get<SiftDBPtr>(this->Voc_DB)->query(DBoW2::FSift128::fromMat32F(frame->KeyPointsDescriptors),
				//	ret, this->MatchingNumber);

				this->m_Siftdb->query(DBoW2::FSift128::fromMat32F(frame->KeyPointsDescriptors),
					ret, this->MatchingNumber);

				//auto id2 = std::get<SiftDBPtr>(this->Voc_DB)->add(DBoW2::FSift128::fromMat32F(frame->KeyPointsDescriptors));
				auto id2=this->m_Siftdb->add(DBoW2::FSift128::fromMat32F(frame->KeyPointsDescriptors));
				this->MatcherIndexMap.insert({ id2,frame });
				break;
			}	
			default:
				break;
			}

			if (ret.empty())
				return false;

			for (auto& i : ret)
			{
				auto ptr = this->MatcherIndexMap.at(i.Id).lock();
				if (ptr != nullptr)
				{
					related.push_back(ptr);
				}
				else
				{
					std::cout << this->type_name() << ": frame dose not Exist, voc remaped id=" << i.Id << std::endl;
				}
			}
			return true;
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	return false;
}

bool VocTreeMatcher::MatchKeyPoints(FrameObject::Ptr frame)
{
	try
	{
		std::set<int> RelatedFrameID;
		frame->getAllRelatedFrames(RelatedFrameID);
		if (RelatedFrameID.empty())
		{
			std::cout << this->type_name() << ": failed to match keypoints" 
				<< frame->getID() << " has no related frame." << std::endl;
			return false;
		}
		
		bool is_found = false;
		for (auto& i : RelatedFrameID)
		{
			auto RelatedFremeInfo = frame->getRelatedFrame(i);
			auto RelateFrame = RelatedFremeInfo->getRelatedFrame();
			std::vector<cv::DMatch> matches;
			auto result = this->MatchKeyPoints(frame, RelateFrame, RelatedFremeInfo->KeyPointMatch);
			if (result == true)
			{
				is_found = true;
			}
		}
		return is_found;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	return false;
}

bool VocTreeMatcher::MatchKeyPoints(FrameObject::Ptr queryFrames, FrameObject::Ptr trainFrames, std::vector<cv::DMatch>& matches)
{
	try
	{
		matches.clear();
		std::vector<cv::DMatch> PremaryMatch;
		this->m_Matcher->clear();
		this->m_Matcher->match(queryFrames->KeyPointsDescriptors, trainFrames->KeyPointsDescriptors, PremaryMatch);
		if (PremaryMatch.empty())
			return false;
		
		cv::xfeatures2d::matchGMS(queryFrames->RGBMat.size(), trainFrames->RGBMat.size(),
			queryFrames->KeyPoints, trainFrames->KeyPoints, PremaryMatch,
			matches, bigRotation, bigScale, GMSthresholdFactor);
		
		return matches.size() >= this->LeastMatchingPoint;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	return false;
}

bool VocTreeMatcher::save(JsonNode& fs)
{
	try
	{
		fs["keypoint-type"] = this->m_KeyPointType;
		fs["voc-tree-path"] = this->VocPath;
		fs["matcher-type"] = this->m_MatcherType;
		
		if (!this->VocPath.empty())
		{
			switch (this->m_KeyPointType)
			{
			case KeyPointType_e::ORB:
				this->m_Orbdb->save(this->VocPath);
				//std::get<OrbDatabase>(this->Voc_DB)->save(this->VocPath);
				break;
			case KeyPointType_e::SIFT:
				this->m_Siftdb->save(this->VocPath);
				//std::get<Sift128Database>(this->Voc_DB)->save(this->VocPath);
				break;
			default:
				std::cout << this->type_name() << ": failed to save database type error" << std::endl;
				break;
			}
		}

		fs["least-matching-points"] = this->LeastMatchingPoint;
		//save gms parameters
		fs["large-rotation"] = this->bigRotation;
		fs["large-scale"] = this->bigScale;
		fs["gms-threshold"] = this->GMSthresholdFactor;

		return true;
	}
	catch (const std::exception& e)
	{
        std::cout << e.what() << std::endl;
	}
	
    return false;
}

bool VocTreeMatcher::load(JsonNode& fs)
{
	try
	{
		this->m_KeyPointType = fs.at("keypoint-type");
		this->VocPath = fs.at("voc-tree-path");
		this->m_MatcherType = fs.at("matcher-type");
		switch (this->m_KeyPointType)
		{
		case KeyPointType_e::ORB:
			this->m_Orbdb = std::make_unique<OrbDatabase>(this->VocPath);
			break;
		case KeyPointType_e::SIFT:
			this->m_Siftdb = std::make_unique<Sift128Database>(this->VocPath);
			break;
		default:
			break;
		}

		this->m_Matcher = cv::DescriptorMatcher::create(static_cast<cv::DescriptorMatcher::MatcherType>(this->m_MatcherType));

		//read parameters
		try
		{
			bigRotation = fs.at("large-rotation");
			bigScale = fs.at("large-scale");
			GMSthresholdFactor = fs.at("gms-threshold");
			this->LeastMatchingPoint = fs.at("least-matching-points");
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what() << std::endl;
			std::cerr << this->type_name() << ": failed to load GMS parameters, default will be used!" << std::endl;
			bigRotation = false;
			bigScale = false;
			GMSthresholdFactor = 6.0;
			this->LeastMatchingPoint = 20;
		}
		
		return this->isInit();
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	
    return false;
}

bool VocTreeMatcher::clear()
{
	this->m_Matcher->clear();
	this->m_Matcher.reset();
	//this->Voc_DB = std::variant<SiftDBPtr, OrbDBPtr>();
	this->VocPath.clear();
    return true;
}

bool VocTreeMatcher::isInit()
{
	if (this->VocPath.empty())
		return false;
	if (this->m_Matcher == nullptr)
		return false;
	
	switch (this->m_KeyPointType)
	{
	case KeyPointType_e::ORB:
		if (this->m_Orbdb == nullptr)
			return false;
		break;
	case KeyPointType_e::SIFT:
		if (this->m_Siftdb == nullptr)
			return false;
		break;
	default:
		std::cout << this->type_name() << ": unknown matcher type" << std::endl;
		return false;
		break;
	}
	
	return true;
}
