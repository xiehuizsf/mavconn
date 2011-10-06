#include "PxOpenCVCameraManager.h"

#include "PxOpenCVCamera.h"
//#include "PxOpenCVStereoCamera.h"

PxOpenCVCameraManager::PxOpenCVCameraManager()
{
}

PxOpenCVCameraManager::~PxOpenCVCameraManager()
{

}

PxCameraPtr
PxOpenCVCameraManager::generateCamera(uint64_t captureIndex)
{
	return PxCameraPtr(new PxOpenCVCamera(getCamera(captureIndex)));
}

PxStereoCameraPtr
PxOpenCVCameraManager::generateStereoCamera(uint64_t serialNum1, uint64_t serialNum2)
{
	fprintf(stderr, "# ERROR: OpenCV cam does not support stereo");
	return PxStereoCameraPtr();
}

int
PxOpenCVCameraManager::getCameraCount(void) const
{
	return 1;
}

cv::VideoCapture*
PxOpenCVCameraManager::getCamera(uint64_t captureIndex)
{
	cv::VideoCapture* cap = new cv::VideoCapture(captureIndex);
	return cap;
}
