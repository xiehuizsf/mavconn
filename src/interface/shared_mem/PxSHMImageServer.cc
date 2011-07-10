/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009-2011 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
* @file
*   @brief Shared memory interface for writing images.
*
*   This interface is a wrapper around PxSHM.
*
*   @author Lionel Heng  <hengli@inf.ethz.ch>
*
*/

#include "PxSHMImageServer.h"

PxSHMImageServer::PxSHMImageServer()
{
	
}
	
bool
PxSHMImageServer::init(int sysid, int compid, lcm_t* lcm,
					   PxSHM::Camera cam1, PxSHM::Camera cam2)
{
	this->sysid = sysid;
	this->compid = compid;
	this->lcm = lcm;
	this->cam1 = cam1;
	this->cam2 = cam2;
	key = cam1 | cam2;
	
	initCameraProperties = false;
	
	img_seq = 0;
	
	return shm.init(key, PxSHM::SERVER_TYPE, 128, 1, 1024 * 1024, 10);
}
	
void
PxSHMImageServer::writeMonoImage(const cv::Mat& img, uint64_t camId,
								 uint64_t timestamp, float roll, float pitch, float yaw,
								 float z, float lon, float lat, float alt,
								 uint32_t exposure)
{
	if (!initCameraProperties)
	{
		writeCameraProperties(img.channels() == 1 ? PxSHM::CAMERA_MONO_8 : PxSHM::CAMERA_MONO_24,
							  img.cols, img.rows, img.type());
		initCameraProperties = false;
	}
	
	std::vector<uint8_t> data;
	int imageSize = img.elemSize() * img.rows * img.cols;
	data.resize(imageSize);
	memcpy(&(data[0]), img.data, imageSize);
	
	shm.writeDataPacket(data);
	
	struct timeval tv;
	gettimeofday(&tv, NULL);
	uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	uint64_t valid_until = now + (uint64_t)(100000);
	
	mavlink_image_available_t imginfo;
	imginfo.cam_id = camId;
	imginfo.cam_no = cam1;
	imginfo.timestamp = timestamp;
	imginfo.valid_until = valid_until;
	imginfo.img_seq = this->img_seq;
	imginfo.img_buf_index = 1;	//FIXME
	imginfo.width = img.cols;
	imginfo.height = img.rows;
	imginfo.depth = img.depth();
	imginfo.channels = img.channels();
	imginfo.key = (int)this->key;
	imginfo.exposure = exposure;
	imginfo.gain = 1;//gain;
	imginfo.roll = roll;
	imginfo.pitch = pitch;
	imginfo.yaw = yaw;
	imginfo.local_z = z;
	imginfo.lon = lon;
	imginfo.lat = lat;
	imginfo.alt = alt;
	
	mavlink_message_t msg;
	mavlink_msg_image_available_encode(this->sysid, this->compid, &msg, &imginfo);
	mavlink_message_t_publish (lcm, "IMAGES", &msg);
	
	img_seq++;
}
	
void
PxSHMImageServer::writeStereoImage(const cv::Mat& imgLeft, uint64_t camIdLeft,
								   const cv::Mat& imgRight, uint64_t camIdRight,
								   uint64_t timestamp, float roll, float pitch, float yaw,
								   float z, float lon, float lat, float alt,
								   uint32_t exposure)
{
	if (!initCameraProperties)
	{
		writeCameraProperties(imgLeft.channels() == 1 ? PxSHM::CAMERA_STEREO_8 : PxSHM::CAMERA_STEREO_24,
							  imgLeft.cols, imgLeft.rows, imgLeft.type());
		initCameraProperties = false;
	}
	
	std::vector<uint8_t> data;
	int imageSize = imgLeft.elemSize() * imgLeft.rows * imgLeft.cols;
	data.resize(imageSize * 2);
	memcpy(&(data[0]), imgLeft.data, imageSize);
	memcpy(&(data[imageSize]), imgRight.data, imageSize);
	
	shm.writeDataPacket(data);
	
	struct timeval tv;
	gettimeofday(&tv, NULL);
	uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	uint64_t valid_until = now + (uint64_t)(100000);
	
	mavlink_image_available_t imginfo;
	imginfo.cam_id = camIdLeft;
	imginfo.cam_no = cam1 | cam2;
	imginfo.timestamp = timestamp;
	imginfo.valid_until = valid_until;
	imginfo.img_seq = this->img_seq;
	imginfo.img_buf_index = 2;	//FIXME
	imginfo.width = imgLeft.cols;
	imginfo.height = imgLeft.rows;
	imginfo.depth = imgLeft.depth();
	imginfo.channels = imgLeft.channels();
	imginfo.key = (int)this->key;
	imginfo.exposure = exposure;
	imginfo.gain = 1;//gain;
	imginfo.roll = roll;
	imginfo.pitch = pitch;
	imginfo.yaw = yaw;
	imginfo.local_z = z;
	imginfo.lon = lon;
	imginfo.lat = lat;
	imginfo.alt = alt;
	
	mavlink_message_t msg;
	mavlink_msg_image_available_encode(this->sysid, this->compid, &msg, &imginfo);
	mavlink_message_t_publish (lcm, "IMAGES", &msg);
	
	img_seq++;
}

void
PxSHMImageServer::writeKinectImage(const cv::Mat& imgBayer, const cv::Mat& imgDepth,
								   uint64_t timestamp, float roll, float pitch, float yaw,
								   float z, float lon, float lat, float alt)
{
	if (!initCameraProperties)
	{
		writeCameraProperties(PxSHM::CAMERA_KINECT,
							  imgBayer.cols, imgBayer.rows, imgBayer.type());
		initCameraProperties = false;
	}

	std::vector<uint8_t> data;
	int imageSize = imgBayer.elemSize() * imgBayer.rows * imgBayer.cols;
	data.resize(imageSize * 3);
	memcpy(&(data[0]), imgBayer.data, imageSize);
	memcpy(&(data[imageSize]), imgDepth.data, imageSize * 2);
	
	shm.writeDataPacket(data);
	
	struct timeval tv;
	gettimeofday(&tv, NULL);
	uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	uint64_t valid_until = now + (uint64_t)(100000);
	
	mavlink_image_available_t imginfo;
	imginfo.cam_id = 0;
	imginfo.cam_no = cam1;
	imginfo.timestamp = timestamp;
	imginfo.valid_until = valid_until;
	imginfo.img_seq = this->img_seq;
	imginfo.img_buf_index = 1;	//FIXME
	imginfo.width = imgBayer.cols;
	imginfo.height = imgBayer.rows;
	imginfo.depth = imgBayer.depth();
	imginfo.channels = imgBayer.channels();
	imginfo.key = (int)this->key;
	imginfo.exposure = 0;
	imginfo.gain = 1;//gain;
	imginfo.roll = roll;
	imginfo.pitch = pitch;
	imginfo.yaw = yaw;
	imginfo.local_z = z;
	imginfo.lon = lon;
	imginfo.lat = lat;
	imginfo.alt = alt;
	
	mavlink_message_t msg;
	mavlink_msg_image_available_encode(this->sysid, this->compid, &msg, &imginfo);
	mavlink_message_t_publish (lcm, "IMAGES", &msg);
	
	img_seq++;
}

bool
PxSHMImageServer::writeCameraProperties(PxSHM::CameraType cameraType,
										int imageWidth, int imageHeight,
										int imageType)
{
	std::vector<uint8_t> data;
	data.resize(16);
	
	memcpy(&(data[0]), &cameraType, 4);
	memcpy(&(data[4]), &imageWidth, 4);
	memcpy(&(data[8]), &imageHeight, 4);
	memcpy(&(data[12]), &imageType, 4);

	if (shm.writeInfoPacket(data) != static_cast<int>(data.size()))
	{
		return false;
	}
	
	return true;
}
