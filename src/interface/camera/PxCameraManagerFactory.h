#ifndef PXCAMERAMANAGERFACTORY_H
#define PXCAMERAMANAGERFACTORY_H

#include <string>

#include "PxCameraManager.h"

class PxCameraManagerFactory
{
public:
	static PxCameraManagerPtr generate(const std::string& type);

private:
	static PxCameraManagerPtr bluefoxCameraManager;
	static PxCameraManagerPtr fireflyCameraManager;
	static PxCameraManagerPtr opencvCameraManager;
};

#endif
