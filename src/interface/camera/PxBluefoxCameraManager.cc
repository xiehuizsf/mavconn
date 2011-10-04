#include "PxBluefoxCameraManager.h"

#include "PxBluefoxCamera.h"
#include "PxBluefoxStereoCamera.h"

PxBluefoxCameraManager::PxBluefoxCameraManager()
{

}

PxCameraPtr
PxBluefoxCameraManager::generateCamera(uint64_t serialNum)
{
	std::ostringstream oss;
	oss << serialNum;

	mvIMPACT::acquire::Device* dev = deviceManager.getDeviceBySerial(oss.str());
	if (dev == 0)
	{
		fprintf(stderr, "# ERROR: Cannot find Bluefox camera with "
						"serial number %lu.\n", serialNum);
		return PxCameraPtr();
	}

	return PxCameraPtr(new PxBluefoxCamera(dev));
}

PxStereoCameraPtr
PxBluefoxCameraManager::generateStereoCamera(uint64_t serialNum1, uint64_t serialNum2)
{
	std::ostringstream oss1;
	oss1 << serialNum1;

	mvIMPACT::acquire::Device* dev1 = deviceManager.getDeviceBySerial(oss1.str());
	if (dev1 == 0)
	{
		fprintf(stderr, "# ERROR: Cannot find Bluefox camera with "
						"serial number %lu.\n", serialNum1);
		return PxStereoCameraPtr();
	}

	std::ostringstream oss2;
	oss2 << serialNum2;

	mvIMPACT::acquire::Device* dev2 = deviceManager.getDeviceBySerial(oss2.str());
	if (dev2 == 0)
	{
		fprintf(stderr, "# ERROR: Cannot find Bluefox camera with "
						"serial number %lu.\n", serialNum2);
		return PxStereoCameraPtr();
	}

	return PxStereoCameraPtr(new PxBluefoxStereoCamera(dev1, dev2));
}

int
PxBluefoxCameraManager::getCameraCount(void) const
{
	return deviceManager.deviceCount();
}
