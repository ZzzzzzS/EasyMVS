#pragma once
#include "FeatureMatcher.h"
#include "FSift128.h"

#include <DBoW2/DBoW2.h>
#include <opencv2/features2d.hpp>
#include <variant>

/**
 * @details
 * **VocabularyTree** It uses image retrieval techniques to find images 
 * that share some content without the cost of resolving all feature 
 * matches in details. Each image is represented in a compact image 
 * descriptor which allows to compute the distance between all images 
 * descriptors very efficiently. If your scene contains less than "Voc Tree:
 *  Minimal Number of Images", all image pairs will be selected.
 */
class VocTreeMatcher : public FeatureMatcher
{
public:
	Q_OBJECT

public:
	using Ptr = std::shared_ptr<VocTreeMatcher>;
	static VocTreeMatcher::Ptr Create(GlobalMapObject::Ptr GlobalMap);

public:
	VocTreeMatcher(GlobalMapObject::Ptr GlobalMap);
	virtual ~VocTreeMatcher();

	bool Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap = GlobalMapObject::Ptr()) override;
	virtual bool MatchRelatedFrame(FrameObject::Ptr frame, std::list<FrameObject::Ptr>& related, GlobalMapObject::Ptr GlobalMap = GlobalMapObject::Ptr()) override;
	virtual bool MatchKeyPoints(FrameObject::Ptr frame) override;
	virtual bool MatchKeyPoints(FrameObject::Ptr queryFrames, FrameObject::Ptr trainFrames, std::vector<cv::DMatch>& matches) override;
	bool save(JsonNode& fs) override;
	bool load(JsonNode& fs) override;
	bool clear() override;
	virtual bool isInit() override;

	/**
	 * @brief the path of vocabulary tree database.
	 */
	std::string VocPath;
private:
	using Sift128Vocabulary = DBoW2::TemplatedVocabulary<DBoW2::FSift128::TDescriptor, DBoW2::FSift128>;
	using Sift128Database = DBoW2::TemplatedDatabase<DBoW2::FSift128::TDescriptor, DBoW2::FSift128>;
	
	std::shared_ptr<Sift128Database> m_Siftdb;
	std::shared_ptr<OrbDatabase> m_Orbdb;
	using SiftDBPtr = std::unique_ptr<Sift128Database>;
	using OrbDBPtr = std::unique_ptr<OrbDatabase>;
	//std::variant<SiftDBPtr, OrbDBPtr> Voc_DB;
	
	enum class KeyPointType_e
	{
		ORB = 0,
		SIFT
	};
	KeyPointType_e m_KeyPointType;
	NLOHMANN_JSON_SERIALIZE_ENUM(KeyPointType_e, {
		{KeyPointType_e::ORB,"orb"},
		{KeyPointType_e::SIFT,"sift"}
	});

	enum class MatcherType_e
	{
		FLANNBASED = cv::DescriptorMatcher::FLANNBASED,
		BRUTEFORCE = cv::DescriptorMatcher::BRUTEFORCE,
		BRUTEFORCE_L1 = cv::DescriptorMatcher::BRUTEFORCE_L1,
		BRUTEFORCE_HAMMING = cv::DescriptorMatcher::BRUTEFORCE_HAMMING,
		BRUTEFORCE_HAMMINGLUT = cv::DescriptorMatcher::BRUTEFORCE_HAMMINGLUT,
		BRUTEFORCE_SL2 = cv::DescriptorMatcher::BRUTEFORCE_SL2,
	};
	MatcherType_e m_MatcherType;
	NLOHMANN_JSON_SERIALIZE_ENUM(MatcherType_e, {
		{MatcherType_e::FLANNBASED,"flann-based"},
		{MatcherType_e::BRUTEFORCE,"brute-force"},
		{MatcherType_e::BRUTEFORCE_L1,"brute-force-l1"},
		{MatcherType_e::BRUTEFORCE_HAMMING,"brute-force-hamming"},
		{MatcherType_e::BRUTEFORCE_HAMMINGLUT,"brute-force-hamming-lut"},
		{MatcherType_e::BRUTEFORCE_SL2,"brute-force-sl2"},
	});
	
	cv::Ptr<cv::DescriptorMatcher> m_Matcher;

	///parameters for keypoints matching details can be found in opencv documents **matchGMS** part
	bool bigRotation;
	bool bigScale;
	double GMSthresholdFactor;
	int LeastMatchingPoint;

	///parameters for image matching
	int MatchingNumber;
	
	// remap dbow2 id with frame id
	std::map<DBoW2::EntryId, std::weak_ptr<FrameObject>> MatcherIndexMap;

};
