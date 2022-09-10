#pragma once
#include <iostream>
#include "MVSConfig.h"
#include <opencv2/opencv.hpp>

/**
 * @brief the base class of all MVS objects
 * 
 */
class MVSObject
{
public:
/**
 * @brief Construct a new MVSObject object
 * 
 */
	MVSObject();

/**
 * @brief Destroy the MVSObject object
 * 
 */
	virtual ~MVSObject(){}

/**
 * @brief save data and settings to file
 * 
 * @details this method will save data such as keypoints, descriptors, translation matrices in DataFlowObject
 * and its sub-classes to the given JsonNode instance, or will save parameters and settings in WorkFlowObject 
 * and its sub-classes to the given JsonNode instance.
 * 
 * @param fs the json handler
 * @return true save successfully
 * @return false save failed
 */
	virtual bool save(JsonNode& fs) = 0;

/**
 * @brief load data and settings from file
 * 
 * @details this method will load data such as keypoints, descriptors, translation matrices in DataFlowObject
 * and its sub-classes from the given JsonNode instance. or will load parameters and settings in WorkFlowObject 
 * and its sub-classes to the given JsonNode instance, **and automatically initialize the WorkFlow with these parameters**
 *
 * @param fs the json handler 
 * @return true load successfully
 * @return false load failed or initialization falied.
 */
	virtual bool load(JsonNode& fs) = 0;
	
	/**
	 * @brief get the typename.
	 * 
	 * @return 
	 */
	virtual std::string type_name();
	
protected:
	/**
	 * @brief check if the directory is exist.
	 * 
	 * @param path directory
	 * @return 
	 */
	bool isDirExist(std::string path);

	/**
	 * @brief create directory in given path.
	 * 
	 * @param path
	 * @return if create succeed or the directory already exist, return true, 
	 * otherwise return false
	 */
	bool mkdir(std::string path);
	
private:

};

