#pragma once

#include "MVSConfig.h"
#include "WorkFlowObject.h"
#include "FrameObject.h"
#include "GlobalMapObject.h"
#include <QObject>


/**
 * @details This workflow will analyze feature matches to understand the geometric relationship
 *  behind all the 2D observations, and infer the rigid scene structure (3D points)
 *  with the pose (position and orientation) and internal calibration of all cameras.
 *  The pipeline is a growing reconstruction process (called incremental SfM):
 *  it first computes an initial two-view reconstruction that is iteratively extended by adding new views.
 */
class PoseReconstructor : public WorkFlowObject
{
public:
	Q_OBJECT
    /**
     * @brief shared pointer type of PoseReconstructor
     * 
     */
	using Ptr = std::shared_ptr<PoseReconstructor>;

    /**
     * @brief create shared pointer of PoseReconstructor
     * 
     * @param GlobalMap the global map pointer
     * @return PoseReconstructor::Ptr the shared pointer type of PoseReconstructor
     */
	static PoseReconstructor::Ptr Create(GlobalMapObject::Ptr GlobalMap);
public:
/**
 * @brief Construct a new Pose Reconstructor object
 * 
 */
	PoseReconstructor(GlobalMapObject::Ptr GlobalMap);

    /**
     * @brief Destroy the Pose Reconstructor object
     * 
     */
	virtual ~PoseReconstructor();

    /**
     * @brief Get the Flow Name
     * 
     * @return std::string flow name
     */
	std::string type_name() override;

	bool clear() override;

    /**
     * @brief compute the pose of the current frame
     * 
     * @param frame inputoutput current frame.
     * @param GlobalMap GlobalMap the global map pointer, if the global map is empty,
     * the method will use preloaded global map.
     * @return true calculate the global pose successfully.
     * @return false failed to calculate the global pose, or input parameter is incorrect.
     */
	virtual bool Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap = GlobalMapObject::Ptr());

    bool save(JsonNode& fs) override;

    bool load(JsonNode& fs) override;

public slots:
/**
 * @warning **this method shold NOT be called since this algorithm can NOT process empty input data.**
 */
	void Trigger() override;

	void Trigger(DataQueue data) override;


protected:
	GlobalMapObject::Ptr GlobalMap;
};
