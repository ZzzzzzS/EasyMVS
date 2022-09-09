#pragma once
#include "MVSObject.h"
#include "MVSConfig.h"
#include "DataFlowObject.h"
#include <QObject>
#include <opencv2/opencv.hpp>
#include <queue>


/**
 * @brief this is the base class of all workflow class,
 * the workflow classes will implement different algorithms to operate data which stored in dataflow classes to complete SFM and MVS steps.
 * 
 */
class WorkFlowObject : public QObject, public MVSObject
{
	Q_OBJECT
public:
/**
 * @brief Construct a new Work Flow Object object
 * 
 */
	WorkFlowObject();

	/**
	 * @brief Destroy the Work Flow Object object
	 * 
	 */
	virtual ~WorkFlowObject() {}
	
public:

	/**
	 * @brief clear the workflow state, the workflow should be re-init after clear.
	 * 
	 * @return true clear succeed
	 * @return false clear failed
	 */
	virtual bool clear() = 0;

/**
 * @brief init the workflow with paramters stored in json node.
 * 
 * @param fs 
 * @return true init succeed
 * @return false init faled
 */
	virtual bool init(JsonNode& fs) = 0;

	/**
	 * @brief check if the workflow is inited
	 * 
	 * @return true inited
	 * @return false not inited
	 */
	virtual bool isInit();

	/**
	 * @brief save the current paramter to json node
	 * 
	 * @param fs 
	 * @return true save succeed
	 * @return false save failed
	 */
	virtual bool saveParameter(JsonNode& fs) = 0;
	
public:
	using DataQueue=std::queue<DataFlowObject::Ptr>;

signals:
/**
 * @brief the current work load of the workflow is finished, 
 * and the processeed data pointer is stored in the queue, 
 * the workflow can be assigned to new work load with **trigger()** methiod
 * 
 */
	void Finished(DataQueue);
/**
 * @brief the current work load of the workflow is finished, the workflow can be assigned to new work load with **trigger()** methiod
 * 
 */
	void Finished();
	void Failed();

	/**
	 * @brief emit error message
	 * 
	 * @param error 
	 */
	void Error(std::string error);

	/**
	 * @brief emit percentage of the work load 0-100
	 * 
	 * @param progress 
	 */
	void Progress(int progress);

	/**
	 * @brief emit warning output message
	 * 
	 * @param warning 
	 */
	void Warning(std::string warning);

	/**
	 * @brief emit information output message
	 * 
	 * @param info 
	 */
	void Info(std::string info);
	
public slots:

/**
 * @brief trigger one work load of the workflow with preload input and output settings.
 * 
 */
	virtual void Trigger() = 0;

	/**
	 * @brief trigger one work load of the workflow with input data.
	 * 
	 * @param data queue of shared pointer of dataflow object(commonly is frame object)
	 */
	virtual void Trigger(DataQueue data) = 0;
	

public:
/**
 * @brief shared pointer of WorkFlowObject
 * 
 */
	using Ptr = std::shared_ptr<WorkFlowObject>;
	
protected:
	bool m_isInit = false;
};

