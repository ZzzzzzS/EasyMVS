#pragma once
#include <iostream>
#include "MVSConfig.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/persistence.hpp>

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
 * @brief save data to file
 * 
 * @param fs the json handler
 * @return true save successfully
 * @return false save failed
 */
	virtual bool save(JsonNode& fs) = 0;

/**
 * @brief load data from file
 * 
 * @param fs the json handler 
 * @return true load successfully
 * @return false load failed
 */
	virtual bool load(JsonNode& fs) = 0;
	
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

