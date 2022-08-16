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
	virtual bool save(Json& fs) = 0;

/**
 * @brief load data from file
 * 
 * @param fs the json handler 
 * @return true load successfully
 * @return false load failed
 */
	virtual bool load(Json& fs) = 0;
	
	
private:

};

