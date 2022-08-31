#pragma once
#include "FeatureMatcher.h"

/**
 * @brief 
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
	using Ptr = std::shared_ptr<VocTreeMatcher>;
	static VocTreeMatcher::Ptr Create(GlobalMapObject::Ptr GlobalMap);
public:
	VocTreeMatcher(GlobalMapObject::Ptr GlobalMap);
	virtual ~VocTreeMatcher();

	bool Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap = GlobalMapObject::Ptr()) override;
	bool saveParameter(JsonNode& fs) override;
	bool init(JsonNode& fs) override;
	bool clear() override;


private:
	//TODO: 在这里设计具体的匹配方法
};
