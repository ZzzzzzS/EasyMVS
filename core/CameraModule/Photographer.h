#pragma once

#include "MVSConfig.h"
#include "WorkFlowObject.h"
#include "CameraObject.h"
#include "FrameObject.h"

#include <QObject>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sophus/se3.hpp>

class Photographer : public WorkFlowObject
{
	Q_OBJECT
public:
/**
 * @brief Photographer shared pointer type
 */
	using Ptr = std::shared_ptr<Photographer>;

	/**
	 * @brief create shared pointer of Photographer
	 * 
	 * @param Cameras camera lists
	 * @return Photographer::Ptr 
	 */
	static Photographer::Ptr Create(std::initializer_list<CameraObject::Ptr> Cameras);
public:
/**
 * @brief Construct a new Photographer object
 * 
 */
	Photographer();

	/**
	 * @brief Construct a new Photographer object with one camera
	 * 
	 * @param Camera camera pointer
	 */
	Photographer(std::shared_ptr<CameraObject::Ptr> Camera);

	/**
	 * @brief Construct a new Photographer object with multiple cameras
	 * 
	 * @param Cameras camera pointer lists
	 */
	Photographer(std::initializer_list<CameraObject::Ptr> Cameras);

	/**
	 * @brief Destroy the Photographer object
	 * 
	 */
	virtual ~Photographer();
	
public:
/**
 * @brief Get the Flow Name
 * 
 * @return std::string flow name
 */
	std::string getFlowName() override;

	/**
	 * @brief initialize the flow with given parameters
	 * 
	 * @param fs the json node contain parameters
	 * @return true initialized successfully
	 * @return false initialized failed
	 */
	bool init(JsonNode& fs) override;

	/**
	 * @brief take a photo with given cameras
	 * 
	 * @param Frames output frames generated by the cameras
	 * @return true succeed
	 * @return false failed
	 */
	virtual bool takePhoto(std::vector<FrameObject::Ptr>& Frames);

	/**
	 * @brief save the parameters of the flow to json node
	 * 
	 * @param fs json node
	 * @return true succeed
	 * @return false failed
	 */
	bool saveParameter(JsonNode& fs) override;

	/**
	 * @brief clear the parameters of the flow, and deinitialize the flow
	 * 
	 * @return true succeed
	 * @return false failed
	 */
	bool clear() override;

public slots:
/**
 * @brief trigger the flow with pre-defined parameters
 * 
 */
	void Trigger() override;

	/**
	 * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
	 * 
	 * @param data input data
	 */
	
	void Trigger(Photographer::DataQueue data);
	
private:
	using CameraGroup = std::tuple<CameraObject::Ptr, Sophus::SE3d>;
	std::vector<CameraGroup> Cameras;
	
	
};
