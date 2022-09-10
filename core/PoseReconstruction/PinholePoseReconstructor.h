#pragma once
#include "PoseReconstructor.h"

/**
 * @details this is the implementation of increasemental sfm(structure from motion)
 * under pinhole camera model secnario.
 */
class PinholePoseReconstructor: public PoseReconstructor
{
public:
	Q_OBJECT
	using Ptr = std::shared_ptr<PinholePoseReconstructor>;
	static PinholePoseReconstructor::Ptr Create(GlobalMapObject::Ptr GlobalMap);

public:
	PinholePoseReconstructor(GlobalMapObject::Ptr GlobalMap);
	virtual ~PinholePoseReconstructor();

	virtual bool save(JsonNode& fs) override;
	virtual bool load(JsonNode& fs) override;
	virtual bool clear() override;
	virtual bool Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap = GlobalMapObject::Ptr()) override;

private:

};
