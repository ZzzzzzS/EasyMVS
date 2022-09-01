#pragma once
#include "Photographer.h"
#include "CameraObject.h"
#include <QObject>

/**
 * @brief DualCamera Photographer class.
 */
class DualPhotographer : public Photographer
{
public:
	Q_OBJECT
		using Ptr = std::shared_ptr<DualPhotographer>;
	static DualPhotographer::Ptr Create(std::initializer_list<CameraObject::Ptr> Cameras);
public:
	DualPhotographer();
	DualPhotographer(std::initializer_list<CameraObject::Ptr> Cameras);
	virtual ~DualPhotographer();

public:
	bool init(JsonNode& fs) override;
	bool Compute(std::vector<FrameObject::Ptr>& Frames) override;
	bool saveParameter(JsonNode& fs) override;
	bool clear() override;

};
