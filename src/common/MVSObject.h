#include <iostream>
#include <MVSConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/persistence.hpp>

class MVSObject
{
public:
	MVSObject();
	virtual ~MVSObject(){}

	virtual bool save(Json& fs) = 0;
	virtual bool load(Json& fn) = 0;
	
	
private:

};

