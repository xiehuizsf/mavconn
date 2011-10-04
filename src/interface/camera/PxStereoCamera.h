#ifndef PXSTEREOCAMERA_H
#define PXSTEREOCAMERA_H

#include <opencv2/core/core.hpp>
#include <tr1/memory>

// forward declaration
class PxCameraConfig;

class PxStereoCamera
{
public:
	PxStereoCamera();
	virtual ~PxStereoCamera();

	virtual bool init(void) = 0;
	virtual void destroy(void) = 0;

	virtual bool setConfig(const PxCameraConfig& config) = 0;

	virtual bool start(void) = 0;
	virtual bool stop(void) = 0;

	virtual bool grabFrame(cv::Mat& imageLeft, cv::Mat& imageRight,
						   uint32_t& skippedFrames, uint32_t& sequenceNum) = 0;
};

typedef std::tr1::shared_ptr<PxStereoCamera> PxStereoCameraPtr;

#endif
