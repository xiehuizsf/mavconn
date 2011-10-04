#ifndef PXFIREFLYSTEREOCAMERA_H
#define PXFIREFLYSTEREOCAMERA_H

#include "PxFireflyCamera.h"
#include "PxStereoCamera.h"

class PxFireflyStereoCamera : public PxStereoCamera
{
public:
	PxFireflyStereoCamera(dc1394camera_t* _cameraLeft, dc1394camera_t* _cameraRight);
	~PxFireflyStereoCamera();

	bool init(void);
	void destroy(void);

	bool setConfig(const PxCameraConfig& config);

	bool start(void);
	bool stop(void);

	bool grabFrame(cv::Mat& imageLeft, cv::Mat& imageRight,
				   uint32_t& skippedFrames, uint32_t& sequenceNum);

private:
	std::tr1::shared_ptr<PxFireflyCamera> cameraLeft;
	std::tr1::shared_ptr<PxFireflyCamera> cameraRight;

	uint64_t serialNumLeft;
	uint64_t serialNumRight;

	bool externalTrigger;
};

#endif
