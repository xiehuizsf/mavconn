#include "PxSHMImageServer.h"

PxSHMImageServer::PxSHMImageServer()
{
	
}
	
bool
PxSHMImageServer::init(int sysid, int compid, lcm_t* lcm,
					   PxSHM::CameraPosition cam1, PxSHM::CameraPosition cam2)
{
	this->sysid = sysid;
	this->compid = compid;
	this->lcm = lcm;
	key = cam1 | cam2;
	
	shm.init(key, PxSHM::SERVER_TYPE, 16, 1, 1024 * 1024, 10);
	
	initCameraProperties = false;
	
	img_seq = 0;
	
	return true;
}
	
void
PxSHMImageServer::writeMonoImage(const cv::Mat& img, uint64_t camId, uint32_t camNo,
								 uint64_t timestamp, float roll, float pitch, float yaw,
								 float z, float lon, float lat, float alt,
								 uint32_t exposure)
{
	if (!initCameraProperties)
	{
		if (img.channels() == 1)
		{
			cameraType = PxSHM::CAMERA_MONO_8;
		}
		else
		{
			cameraType = PxSHM::CAMERA_MONO_24;
		}
		imageWidth = img.cols;
		imageHeight = img.rows;
		imageType = img.type();		
		
		writeCameraProperties();
		initCameraProperties = true;
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
	imginfo.cam_no = camNo;
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
PxSHMImageServer::writeStereoImage(const cv::Mat& imgLeft, uint64_t camIdLeft, uint32_t camNoLeft,
								   const cv::Mat& imgRight, uint64_t camIdRight, uint32_t camNoRight,
								   uint64_t timestamp, float roll, float pitch, float yaw,
								   float z, float lon, float lat, float alt,
								   uint32_t exposure)
{
	if (!initCameraProperties)
	{
		if (imgLeft.channels() == 1)
		{
			cameraType = PxSHM::CAMERA_STEREO_8;
		}
		else
		{
			cameraType = PxSHM::CAMERA_STEREO_24;
		}
		imageWidth = imgLeft.cols;
		imageHeight = imgLeft.rows;
		imageType = imgLeft.type();		
		
		writeCameraProperties();
		initCameraProperties = true;
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
	imginfo.cam_no = camNoLeft;
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
		cameraType = PxSHM::CAMERA_KINECT;
		imageWidth = imgBayer.cols;
		imageHeight = imgBayer.rows;
		imageType = imgBayer.type();		
		
		writeCameraProperties();
		initCameraProperties = true;
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
	imginfo.cam_no = 40;
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
PxSHMImageServer::writeCameraProperties(void)
{
	std::vector<uint8_t> data;
	data.resize(16);
	
	memcpy(&(data[0]), &cameraType, 4);
	memcpy(&(data[4]), &imageWidth, 4);
	memcpy(&(data[8]), &imageHeight, 4);
	memcpy(&(data[12]), &imageType, 4);
	
	if (shm.writeInfoPacket(data) <= 0)
	{
		return false;
	}
	
	return true;
}
