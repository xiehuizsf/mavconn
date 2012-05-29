#ifndef PXBLUEFOXSTEREOCAMERA_H
#define PXBLUEFOXSTEREOCAMERA_H

#include "PxBluefoxCamera.h"
#include "PxStereoCamera.h"

class PxBluefoxStereoCamera : public PxStereoCamera
{
public:
	PxBluefoxStereoCamera(mvIMPACT::acquire::Device* _cameraLeft,
						  mvIMPACT::acquire::Device* _cameraRight);
	~PxBluefoxStereoCamera();

	bool init(void);
	void destroy(void);

	bool setConfig(const PxCameraConfig& config);

	bool start(void);
	bool stop(void);

	bool grabFrame(cv::Mat& imageLeft, cv::Mat& imageRight,
				   uint32_t& skippedFrames, uint32_t& sequenceNum);

private:
	enum Flag
	{
		LEFT_IMAGE_AVAILABLE  = 0x01,
		RIGHT_IMAGE_AVAILABLE = 0x02,
		ALL_IMAGES_AVAILABLE  = 0x03
	};

	void stereoImageHandler(void);

	std::tr1::shared_ptr<PxBluefoxCamera> cameraLeft;
	std::tr1::shared_ptr<PxBluefoxCamera> cameraRight;

	Glib::Thread* stereoImageThread;
	int imageAvailable;
	Glib::Mutex imageMutex;
	std::tr1::shared_ptr<Glib::Cond> imageAvailableCond;
	bool exitImageThreads;

	float frameRate;
	int timeout_ms;

	uint64_t serialNumLeft;
	uint64_t serialNumRight;
	uint32_t lastSequenceNum;

	PxCameraConfig::Mode mode;
	bool externalTrigger;
	int maxRequests;
};

#endif
