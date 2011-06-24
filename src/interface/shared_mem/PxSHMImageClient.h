#ifndef PXSHMIMAGECLIENT_H
#define PXSHMIMAGECLIENT_H

#include <mavconn.h>
#include <opencv2/core/core.hpp>

#include "PxSHM.h"

class PxSHMImageClient
{
public:
	PxSHMImageClient();
	
	bool init(bool subscribeLatest,
			  PxSHM::CameraPosition cam1,
			  PxSHM::CameraPosition cam2 = PxSHM::CAMERA_NONE);
	
	static uint64_t getTimestamp(const mavlink_message_t* msg);
	static uint64_t getValidUntil(const mavlink_message_t* msg);
	static uint64_t getCameraID(const mavlink_message_t* msg);
	static int getCameraNo(const mavlink_message_t* msg);
	static bool getRollPitch(const mavlink_message_t* msg, float& roll, float& pitch);
	static bool getRollPitchYaw(const mavlink_message_t* msg, float& roll, float& pitch, float& yaw);
	static bool getLocalHeight(const mavlink_message_t* msg, float& height);
	static bool getGPS(const mavlink_message_t* msg, float& lat, float& lon, float& alt);
	
	bool readMonoImage(const mavlink_message_t* msg, cv::Mat& img);
	bool readStereoImage(const mavlink_message_t* msg, cv::Mat& imgLeft, cv::Mat& imgRight);
	bool readKinectImage(const mavlink_message_t* msg, cv::Mat& imgBayer, cv::Mat& imgDepth);

private:	
	bool readCameraProperties(void);
	
	bool subscribeLatest;
	
	PxSHM shm;
	
	bool initCameraProperties;
	PxSHM::CameraType cameraType;
	int imageWidth;
	int imageHeight;
	int imageType;
};

#endif
