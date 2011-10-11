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
	cv::VideoCapture *cap = getCamera(captureIndex);
	if(cap->isOpened())
	{
		return PxCameraPtr(new PxOpenCVCamera(cap));
	}
	else
	{
		fprintf(stderr, "# ERROR: No OpenCV camera found.\n");
		return PxCameraPtr();
	}
}

PxStereoCameraPtr
PxOpenCVCameraManager::generateStereoCamera(uint64_t serialNum1, uint64_t serialNum2)
{
	fprintf(stderr, "# ERROR: OpenCV camera does not support stereo.\n");
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
