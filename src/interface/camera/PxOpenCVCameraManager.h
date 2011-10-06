#ifndef PXOPENCVCAMERAMANAGER_H
#define PXOPENCVCAMERAMANAGER_H

#include "PxCameraManager.h"

namespace cv
{
class VideoCapture;
}

class PxOpenCVCameraManager : public PxCameraManager
{
public:
	PxOpenCVCameraManager();
	~PxOpenCVCameraManager();

	PxCameraPtr generateCamera(uint64_t captureIndex);
	PxStereoCameraPtr generateStereoCamera(uint64_t captureIndex1, uint64_t captureIndex2);

	int getCameraCount(void) const;

private:
	cv::VideoCapture* getCamera(uint64_t captureIndex);
};

#endif
