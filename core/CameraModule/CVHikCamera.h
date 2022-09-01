#pragma once
#include "CameraObject.h"

class CVHikCamera : public CameraObject
{
public:
	CVHikCamera();
	virtual ~CVHikCamera();

private:
	//TODO: 根据opencv文档中对videoIO设备的定义来编写驱动
};