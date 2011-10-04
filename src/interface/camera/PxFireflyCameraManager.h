#ifndef PXFIREFLYCAMERAMANAGER_H
#define PXFIREFLYCAMERAMANAGER_H

#include <dc1394.h>

#include "PxCameraManager.h"

class PxFireflyCameraManager : public PxCameraManager
{
public:
	PxFireflyCameraManager();
	~PxFireflyCameraManager();

	PxCameraPtr generateCamera(uint64_t serialNum);
	PxStereoCameraPtr generateStereoCamera(uint64_t serialNum1, uint64_t serialNum2);

	int getCameraCount(void) const;

private:
	dc1394camera_t* getCamera(uint64_t serialNum);

	dc1394_t* context;
};

#endif
