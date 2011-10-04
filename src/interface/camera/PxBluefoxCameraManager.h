#ifndef PXBLUEFOXCAMERAMANAGER_H
#define PXBLUEFOXCAMERAMANAGER_H

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

#include "PxCameraManager.h"

class PxBluefoxCameraManager : public PxCameraManager
{
public:
	PxBluefoxCameraManager();

	PxCameraPtr generateCamera(uint64_t serialNum);
	PxStereoCameraPtr generateStereoCamera(uint64_t serialNum1, uint64_t serialNum2);

	int getCameraCount(void) const;

private:
	mvIMPACT::acquire::DeviceManager deviceManager;
};

#endif
