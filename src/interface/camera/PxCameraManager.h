#ifndef PXCAMERAMANAGER_H
#define PXCAMERAMANAGER_H

#include "PxCamera.h"
#include "PxStereoCamera.h"

class PxCameraManager
{
public:
	virtual PxCameraPtr generateCamera(uint64_t serialNum) = 0;
	virtual PxStereoCameraPtr generateStereoCamera(uint64_t serialNum1, uint64_t serialNum2) = 0;

	virtual int getCameraCount(void) const = 0;
};

typedef std::tr1::shared_ptr<PxCameraManager> PxCameraManagerPtr;

#endif
