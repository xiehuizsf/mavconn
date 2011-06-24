#include "PxSHMImageClient.h"

PxSHMImageClient::PxSHMImageClient()
{
	
}

bool
PxSHMImageClient::init(bool subscribeLatest,
					   PxSHM::CameraPosition cam1, PxSHM::CameraPosition cam2)
{
	initCameraProperties = false;
	this->subscribeLatest = subscribeLatest;
	
	shm.init(cam1 | cam2, PxSHM::CLIENT_TYPE, 16, 1, 1024 * 1024, 10);
	
	return true;
}

uint64_t
PxSHMImageClient::getTimestamp(const mavlink_message_t* msg)
{
	// Decode message
	if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		// Instantly return if MAVLink message did not contain an image
		return 0;
	}
	else
	{
		// Extract the image meta information and pointer location from the image
		mavlink_image_available_t img;
		mavlink_msg_image_available_decode(msg, &img);
		return img.timestamp;
	}
}

uint64_t
PxSHMImageClient::getValidUntil(const mavlink_message_t* msg)
{
	// Decode message
	if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		// Instantly return if MAVLink message did not contain an image
		return 0;
	}
	else
	{
		// Extract the image meta information and pointer location from the image
		mavlink_image_available_t img;
		mavlink_msg_image_available_decode(msg, &img);
		return img.valid_until;
	}
}

uint64_t
PxSHMImageClient::getCameraID(const mavlink_message_t* msg)
{
	// Decode message
	if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		// Instantly return if MAVLink message did not contain an image
		return 0;
	}
	else
	{
		// Extract the image meta information and pointer location from the image
		mavlink_image_available_t img;
		mavlink_msg_image_available_decode(msg, &img);
		return img.cam_id;
	}
}

int
PxSHMImageClient::getCameraNo(const mavlink_message_t* msg)
{
	// Decode message
	if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		// Instantly return if MAVLink message did not contain an image
		return -1;
	}
	else
	{
		// Extract the image meta information and pointer location from the image
		mavlink_image_available_t img;
		mavlink_msg_image_available_decode(msg, &img);
		return img.cam_no;
	}
}

bool
PxSHMImageClient::getRollPitch(const mavlink_message_t* msg, float& roll, float& pitch)
{
	// Decode message
	if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		// Instantly return if MAVLink message did not contain an image
		return false;
	}
	else
	{
		// Extract the image meta information and pointer location from the image
		mavlink_image_available_t img;
		mavlink_msg_image_available_decode(msg, &img);

		roll = img.roll;
		pitch = img.pitch;

		return true;
	}
}

bool
PxSHMImageClient::getRollPitchYaw(const mavlink_message_t* msg, float& roll, float& pitch, float& yaw)
{
	// Decode message
	if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		// Instantly return if MAVLink message did not contain an image
		return false;
	}
	else
	{
		// Extract the image meta information and pointer location from the image
		mavlink_image_available_t img;
		mavlink_msg_image_available_decode(msg, &img);

		roll = img.roll;
		pitch = img.pitch;
		yaw = img.yaw;

		return true;
	}
}

bool
PxSHMImageClient::getLocalHeight(const mavlink_message_t* msg, float& height)
{
	// Decode message
	if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		// Instantly return if MAVLink message did not contain an image
		return false;
	}
	else
	{
		// Extract the image meta information and pointer location from the image
		mavlink_image_available_t img;
		mavlink_msg_image_available_decode(msg, &img);

		height = img.local_z;

		return true;
	}
}

bool
PxSHMImageClient::getGPS(const mavlink_message_t* msg, float& lat, float& lon, float& alt)
{
	// Decode message
	if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		// Instantly return if MAVLink message did not contain an image
		return false;
	}
	else
	{
		// Extract the image meta information and pointer location from the image
		mavlink_image_available_t img;
		mavlink_msg_image_available_decode(msg, &img);

		lat = img.lat;
		lon = img.lon;
		alt = img.alt;

		return true;
	}
}

bool
PxSHMImageClient::readMonoImage(const mavlink_message_t* msg, cv::Mat& img)
{
	if (!initCameraProperties)
	{
		readCameraProperties();
		initCameraProperties = true;
	}
	
	if (!(cameraType == PxSHM::CAMERA_MONO_8 || cameraType == PxSHM::CAMERA_MONO_24))
	{
		fprintf(stderr, "# WARNING: Images of a different type are stored in shared memory. Aborting...\n");
		return false;
	}
	
	if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		// Instantly return if MAVLink message did not contain an image
		return false;
	}
	
	if (!shm.bytesWaiting())
	{
		return false;
	}
	
	do
	{
		std::vector<uint8_t> data;
		if (shm.readDataPacket(data) <= 0)
		{
			return false;
		}
		
		cv::Mat temp(imageHeight, imageWidth, imageType, &(data[0]));
		temp.copyTo(img);
	}
	while (shm.bytesWaiting() && subscribeLatest);
	
	return true;
}

bool
PxSHMImageClient::readStereoImage(const mavlink_message_t* msg, cv::Mat& imgLeft, cv::Mat& imgRight)
{
	if (!initCameraProperties)
	{
		readCameraProperties();
		initCameraProperties = true;
	}
	
	if (!(cameraType == PxSHM::CAMERA_STEREO_8 || cameraType == PxSHM::CAMERA_STEREO_24))
	{
		fprintf(stderr, "# WARNING: Images of a different type are stored in shared memory. Aborting...\n");
		return false;
	}
	
	if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		// Instantly return if MAVLink message did not contain an image
		return false;
	}
	
	if (!shm.bytesWaiting())
	{
		return false;
	}
	
	do
	{
		std::vector<uint8_t> data;
		if (shm.readDataPacket(data) <= 0)
		{
			return false;
		}
		
		cv::Mat temp1(imageHeight, imageWidth, imageType, &(data[0]));
		temp1.copyTo(imgLeft);
		
		int offset = imgLeft.elemSize() * imageHeight * imageWidth;
		cv::Mat temp2(imageHeight, imageWidth, imageType, &(data[offset]));
		temp2.copyTo(imgRight);
	}
	while (shm.bytesWaiting() && subscribeLatest);
	
	return true;
}

bool
PxSHMImageClient::readKinectImage(const mavlink_message_t* msg, cv::Mat& imgBayer, cv::Mat& imgDepth)
{
	if (!initCameraProperties)
	{
		readCameraProperties();
		initCameraProperties = true;
	}
	
	if (cameraType != PxSHM::CAMERA_KINECT)
	{
		fprintf(stderr, "# WARNING: Images of a different type are stored in shared memory. Aborting...\n");
		return false;
	}
	
	if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		// Instantly return if MAVLink message did not contain an image
		return false;
	}
	
	if (!shm.bytesWaiting())
	{
		return false;
	}
	
	do
	{
		std::vector<uint8_t> data;
		if (shm.readDataPacket(data) <= 0)
		{
			return false;
		}
		
		cv::Mat temp1(imageHeight, imageWidth, CV_8UC1, &(data[0]));
		temp1.copyTo(imgBayer);
		
		int offset = imgBayer.elemSize() * imageHeight * imageWidth;
		cv::Mat temp2(imageHeight, imageWidth, CV_16UC1, &(data[offset]));
		temp2.copyTo(imgDepth);
	}
	while (shm.bytesWaiting() && subscribeLatest);
	
	return true;
}

bool
PxSHMImageClient::readCameraProperties(void)
{
	std::vector<uint8_t> data;
	if (shm.readInfoPacket(data) <= 0)
	{
		return false;
	}
	
	if (data.size() != 16)
	{
		return false;
	}
	
	memcpy(&cameraType, &(data[0]), 4);
	memcpy(&imageWidth, &(data[4]), 4);
	memcpy(&imageHeight, &(data[8]), 4);
	memcpy(&imageType, &(data[12]), 4);
	
	return true;
}
