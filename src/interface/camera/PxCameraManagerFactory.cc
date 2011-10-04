#include "PxCameraManagerFactory.h"

#include "PxBluefoxCameraManager.h"
#include "PxFireflyCameraManager.h"

PxCameraManagerPtr PxCameraManagerFactory::bluefoxCameraManager;
PxCameraManagerPtr PxCameraManagerFactory::fireflyCameraManager;

PxCameraManagerPtr
PxCameraManagerFactory::generate(const std::string& type)
{
	// return singleton instance of device-specific camera manager
	if (type.compare("bluefox") == 0)
	{
		if (bluefoxCameraManager.get() == 0)
		{
			bluefoxCameraManager = PxCameraManagerPtr(new PxBluefoxCameraManager);
		}
		return bluefoxCameraManager;
	}
	else if (type.compare("firefly") == 0)
	{
		if (fireflyCameraManager.get() == 0)
		{
			fireflyCameraManager = PxCameraManagerPtr(new PxFireflyCameraManager);
		}
		return fireflyCameraManager;
	}
	else
	{
		return PxCameraManagerPtr();
	}
}
