#ifndef PXSHMIMAGESERVER_H
#define PXSHMIMAGESERVER_H

#include <mavconn.h>
#include <opencv2/core/core.hpp>

#include "PxSHM.h"

class PxSHMImageServer
{
public:
	PxSHMImageServer();
	
	bool init(int sysid, int compid, lcm_t* lcm,
			  PxSHM::CameraPosition cam1,
			  PxSHM::CameraPosition cam2 = PxSHM::CAMERA_NONE);
	
	void writeMonoImage(const cv::Mat& img, uint64_t camId, uint32_t camNo,
						uint64_t timestamp, float roll, float pitch, float yaw,
						float z, float lon, float lat, float alt,
						uint32_t exposure);
	
	void writeStereoImage(const cv::Mat& imgLeft, uint64_t camIdLeft, uint32_t camNoLeft,
						  const cv::Mat& imgRight, uint64_t camIdRight, uint32_t camNoRight,
						  uint64_t timestamp, float roll, float pitch, float yaw,
						  float z, float lon, float lat, float alt,
						  uint32_t exposure);
	
	void writeKinectImage(const cv::Mat& imgBayer, const cv::Mat& imgDepth,
						  uint64_t timestamp, float roll, float pitch, float yaw,
						  float z, float lon, float lat, float alt);
	
private:
	bool writeCameraProperties(void);
	
	int sysid;
	int compid;
	lcm_t* lcm;
	
	PxSHM shm;
	int key;
	
	bool initCameraProperties;
	PxSHM::CameraType cameraType;
	int imageWidth;
	int imageHeight;
	int imageType;
	
	unsigned int img_seq;
};

#endif
