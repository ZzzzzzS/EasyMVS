#pragma once
#include "DenseReconstructor.h"

class DualFrameReconstructor : public DenseReconstructor
{
public:
	Q_OBJECT
	using Ptr = std::shared_ptr<DualFrameReconstructor>;
	static DualFrameReconstructor::Ptr Create(GlobalMapObject::Ptr GlobalMap);
public:
	DualFrameReconstructor(GlobalMapObject::Ptr GlobalMap);
	virtual ~DualFrameReconstructor();
	bool clear() override;
	bool Compute(FrameObject::Ptr frame) override;
	bool Compute(FrameObject::Ptr frame1, FrameObject Frame2) override;
	
private:

};