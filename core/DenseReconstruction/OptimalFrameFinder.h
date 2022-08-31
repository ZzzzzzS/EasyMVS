#pragma once
#include "MVSConfig.h"
#include "WorkFlowObject.h"
#include "GlobalMapObject.h"
#include "FrameObject.h"

#include <QObject>

class OptimalFrameFinder : public WorkFlowObject
{
public:
	Q_OBJECT
	using Ptr = std::shared_ptr<OptimalFrameFinder>;
	static OptimalFrameFinder::Ptr Create(GlobalMapObject::Ptr GlobalMap);
	
public:
	OptimalFrameFinder(GlobalMapObject::Ptr GlobalMap);
	virtual ~OptimalFrameFinder();
	
public:
	virtual bool clear() override;
	virtual bool init(JsonNode& fs) override;
	virtual bool saveParameter(JsonNode& fs) override;
	
	bool save(JsonNode& fs) override;
	bool load(JsonNode& fs) override;
	
	/**
	 * @brief compute the best frame with the preload settings and preload related frames.
	 * 
	 * @param frame the frame to find the best frame pair.
	 * @return true compute success
	 * @return false failed to find best frame, may because input error such as no preloaded related frame,
	 *  or all related frames are not met the algorithm requirements.
	 */
	virtual bool Compute(FrameObject::Ptr frame);

	/**
	 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts..
	 * 
	 * @param frame the frame to find best frame.
	 * @param RelatedFrame the candidate frame, one of the frame in this frame vector will be the best frame if compute success.
	 * @return 
	 */
	virtual bool Compute(FrameObject::Ptr frame, std::vector<FrameObject::Ptr>& RelatedFrame);

public slots:

	virtual void Trigger(DataQueue data) override;
	virtual void Trigger() override;
	
	

protected:
	GlobalMapObject::Ptr GlobalMap;
};