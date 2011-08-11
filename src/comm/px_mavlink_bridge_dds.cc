#include <mavconn.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dds/Middleware.h"
#include "dds/interface/image/image_interface.h"
#include "dds/interface/mavlink/mavlink_interface.h"
#include "../interface/shared_mem/PxSHMImageClient.h"
#include "../interface/shared_mem/PxSHMImageServer.h"

bool verbose = false;
bool quit = false;

std::vector<PxSHMImageServer> imageServerVec;
std::vector<PxSHMImageClient> imageClientVec;

dds_image_message_t dds_image_msg;

void signalHandler(int signal)
{
	if (signal == SIGINT)
	{
		fprintf(stderr, "# INFO: Shutting down...\n");
		quit = true;
	}
}

void
imageLCMHandler(const lcm_recv_buf_t* rbuf, const char* channel,
				const mavlink_message_t* msg, void* user)
{
	for (size_t i = 0; i < imageClientVec.size(); ++i)
	{
		PxSHMImageClient& client = imageClientVec.at(i);

		if ((client.getCameraConfig() & client.getCameraNo(msg)) != client.getCameraNo(msg))
		{
			continue;
		}

		bool publishImage = false;
		PxSHM::CameraType cameraType;

		std::vector<int> compressionParams;
		compressionParams.push_back(CV_IMWRITE_JPEG_QUALITY);
		compressionParams.push_back(50);  // 50% quality
		std::vector<uchar> buffer;

		// read mono image data
		cv::Mat img;
		if (client.readMonoImage(msg, img))
		{
			dds_image_msg.cols = img.cols;
			dds_image_msg.rows = img.rows;
			dds_image_msg.step1 = img.step[0];
			dds_image_msg.type1 = img.type();

			cv::imencode(".jpg", img, buffer, compressionParams);
			dds_image_msg.imageData1.from_array(reinterpret_cast<DDS_Char*>(&buffer[0]), buffer.size());

			dds_image_msg.step2 = 0;
			dds_image_msg.type2 = 0;
			dds_image_msg.imageData2.length(0);

			if (img.channels() == 1)
			{
				cameraType = PxSHM::CAMERA_MONO_8;
			}
			else
			{
				cameraType = PxSHM::CAMERA_MONO_24;
			}

			publishImage = true;
		}

		cv::Mat imgLeft, imgRight;
		if (client.readStereoImage(msg, imgLeft, imgRight))
		{
			dds_image_msg.cols = imgLeft.cols;
			dds_image_msg.rows = imgLeft.rows;
			dds_image_msg.step1 = imgLeft.step[0];
			dds_image_msg.type1 = imgLeft.type();

			cv::imencode(".png", imgLeft, buffer, compressionParams);
			dds_image_msg.imageData1.from_array(reinterpret_cast<DDS_Char*>(&buffer[0]), buffer.size());

			dds_image_msg.step2 = imgRight.step[0];
			dds_image_msg.type2 = imgRight.type();

			cv::imencode(".png", imgRight, buffer, compressionParams);
			dds_image_msg.imageData2.from_array(reinterpret_cast<DDS_Char*>(&buffer[0]), buffer.size());

			if (imgLeft.channels() == 1)
			{
				cameraType = PxSHM::CAMERA_STEREO_8;
			}
			else
			{
				cameraType = PxSHM::CAMERA_STEREO_24;
			}

			publishImage = true;
		}

		// read Kinect data
		cv::Mat imgBayer, imgDepth;
		if (client.readKinectImage(msg, imgBayer, imgDepth))
		{
			dds_image_msg.cols = imgBayer.cols;
			dds_image_msg.rows = imgBayer.rows;
			dds_image_msg.step1 = imgBayer.step[0];
			dds_image_msg.type1 = imgBayer.type();

			cv::imencode(".png", imgBayer, buffer, compressionParams);
			dds_image_msg.imageData1.from_array(reinterpret_cast<DDS_Char*>(&buffer[0]), buffer.size());

			dds_image_msg.step2 = imgDepth.step[0];
			dds_image_msg.type2 = imgDepth.type();

			cv::imencode(".png", imgDepth, buffer, compressionParams);
			dds_image_msg.imageData2.from_array(reinterpret_cast<DDS_Char*>(&buffer[0]), buffer.size());

			cameraType = PxSHM::CAMERA_KINECT;

			publishImage = true;
		}

		if (publishImage)
		{
			dds_image_msg.camera_config = client.getCameraConfig();
			dds_image_msg.camera_type = cameraType;

			dds_image_msg.cam_id1 = PxSHMImageClient::getCameraID(msg);
			dds_image_msg.timestamp = PxSHMImageClient::getTimestamp(msg);
			PxSHMImageClient::getRollPitchYaw(msg, dds_image_msg.roll, dds_image_msg.pitch, dds_image_msg.yaw);
			PxSHMImageClient::getLocalHeight(msg, dds_image_msg.z);
			PxSHMImageClient::getGPS(msg, dds_image_msg.lon, dds_image_msg.lat, dds_image_msg.alt);
			PxSHMImageClient::getGroundTruth(msg, dds_image_msg.ground_x, dds_image_msg.ground_y, dds_image_msg.ground_z);

			// publish image to DDS
			px::ImageTopic::instance()->publish(&dds_image_msg);

			if (verbose)
			{
				fprintf(stderr, "# INFO: Forwarded image from LCM to DDS.\n");
			}
		}
	}
}

void
mavlinkLCMHandler(const lcm_recv_buf_t* rbuf, const char* channel,
				  const mavlink_message_t* msg, void* user)
{
	if (msg->msgid == MAVLINK_MSG_ID_IMAGE_AVAILABLE)
	{
		return;
	}

	// forward MAVLINK messages from LCM to DDS
	dds_mavlink_message_t dds_msg;
	dds_mavlink_message_t_initialize(&dds_msg);

	dds_msg.len = msg->len;
	dds_msg.seq = msg->seq;
	dds_msg.sysid = msg->sysid;
	dds_msg.compid = msg->compid;
	dds_msg.msgid = msg->msgid;
	memcpy(dds_msg.payload, msg->payload, 255);
	dds_msg.ck_a = msg->ck_a;
	dds_msg.ck_b = msg->ck_b;

	px::MavlinkTopic::instance()->publish(&dds_msg);

	if (verbose)
	{
		fprintf(stderr, "# INFO: Forwarded MAVLINK message [%d] from LCM to DDS.\n", msg->msgid);
	}

	dds_mavlink_message_t_finalize(&dds_msg);
}

void
imageDDSHandler(void* msg)
{
	dds_image_message_t* dds_msg = reinterpret_cast<dds_image_message_t*>(msg);

	int serverIdx = -1;
	for (size_t i = 0; i < imageServerVec.size(); ++i)
	{
		PxSHMImageServer& server = imageServerVec.at(i);
		if (server.getCameraConfig() == dds_msg->camera_config)
		{
			serverIdx = i;
			break;
		}
	}

	if (serverIdx == -1)
	{
		return;
	}

	PxSHMImageServer& server = imageServerVec.at(serverIdx);

	// write image(s) to shared memory
	if (dds_msg->camera_type == PxSHM::CAMERA_MONO_8 ||
		dds_msg->camera_type == PxSHM::CAMERA_MONO_24)
	{
		cv::Mat buffer(dds_msg->imageData1.length(), 1, CV_8U, dds_msg->imageData1.get_contiguous_buffer());
		cv::Mat img = cv::imdecode(buffer, -1);

		server.writeMonoImage(img, dds_msg->cam_id1, dds_msg->timestamp,
							  dds_msg->roll, dds_msg->pitch, dds_msg->yaw,
							  dds_msg->z,
							  dds_msg->lon, dds_msg->lat, dds_msg->alt,
							  dds_msg->ground_x, dds_msg->ground_y, dds_msg->ground_z,
							  dds_msg->exposure);
	}
	else if (dds_msg->camera_type == PxSHM::CAMERA_STEREO_8 ||
			 dds_msg->camera_type == PxSHM::CAMERA_STEREO_24)
	{
		cv::Mat buffer(dds_msg->imageData1.length(), 1, CV_8U, dds_msg->imageData1.get_contiguous_buffer());
		cv::Mat imgLeft = cv::imdecode(buffer, -1);
		buffer = cv::Mat(dds_msg->imageData2.length(), 1, CV_8U, dds_msg->imageData2.get_contiguous_buffer());
		cv::Mat imgRight = cv::imdecode(buffer, -1);

		server.writeStereoImage(imgLeft, dds_msg->cam_id1, imgRight, 0,
								dds_msg->timestamp,
								dds_msg->roll, dds_msg->pitch, dds_msg->yaw,
								dds_msg->z,
								dds_msg->lon, dds_msg->lat, dds_msg->alt,
								dds_msg->ground_x, dds_msg->ground_y, dds_msg->ground_z,
								dds_msg->exposure);
	}
	else if (dds_msg->camera_type == PxSHM::CAMERA_KINECT)
	{
		cv::Mat buffer(dds_msg->imageData1.length(), 1, CV_8U, dds_msg->imageData1.get_contiguous_buffer());
		cv::Mat imgBayer = cv::imdecode(buffer, -1);
		buffer = cv::Mat(dds_msg->imageData2.length(), 1, CV_8U, dds_msg->imageData2.get_contiguous_buffer());
		cv::Mat imgDepth = cv::imdecode(buffer, -1);

		server.writeKinectImage(imgBayer, imgDepth, dds_msg->timestamp,
								dds_msg->roll, dds_msg->pitch, dds_msg->yaw,
								dds_msg->z,
								dds_msg->lon, dds_msg->lat, dds_msg->alt,
								dds_msg->ground_x, dds_msg->ground_y, dds_msg->ground_z);
	}

	if (verbose)
	{
		fprintf(stderr, "# INFO: Forwarded image from DDS to LCM.\n");
	}
}

void
mavlinkDDSHandler(void* msg, lcm_t* lcm)
{
	dds_mavlink_message_t* dds_msg = reinterpret_cast<dds_mavlink_message_t*>(msg);

	// forward MAVLINK messages from DDS to LCM
	mavlink_message_t lcm_msg;

	lcm_msg.len = dds_msg->len;
	lcm_msg.seq = dds_msg->seq;
	lcm_msg.sysid = dds_msg->sysid;
	lcm_msg.compid = dds_msg->compid;
	lcm_msg.msgid = dds_msg->msgid;
	memcpy(lcm_msg.payload, dds_msg->payload, 255);
	lcm_msg.ck_a = dds_msg->ck_a;
	lcm_msg.ck_b = dds_msg->ck_b;

	mavlink_message_t_publish(lcm, "MAVLINK", &lcm_msg);

	if (verbose)
	{
		fprintf(stderr, "# INFO: Forwarded MAVLINK message [%d] from DDS to LCM.\n", dds_msg->msgid);
	}
}

int
main(int argc, char** argv)
{
	enum
	{
		LCM_TO_DDS,
		DDS_TO_LCM
	};

	// Find option for bridge mode
	Glib::OptionGroup optGroup("options", "options", "Configuration options");

	Glib::OptionEntry optBridgeMode;
	optBridgeMode.set_short_name('m');
	optBridgeMode.set_long_name("mode");
	optBridgeMode.set_description("dds2lcm: Push DDS messages to LCM, lcm2dds: Push LCM messages to DDS");

	Glib::OptionEntry optVerbose;
	optVerbose.set_short_name('v');
	optVerbose.set_long_name("verbose");
	optVerbose.set_description("Verbose output");

//	Glib::OptionEntry optProfile;
//	optProfile.set_short_name('p');
//	optProfile.set_long_name("profile");
//	optProfile.set_description("Path to DDS QoS profile file");

	std::string bridgeMode;
	optGroup.add_entry_filename(optBridgeMode, bridgeMode);
	optGroup.add_entry(optVerbose, verbose);

	Glib::OptionContext optContext("");
	optContext.set_help_enabled(true);
	optContext.set_ignore_unknown_options(true);
	optContext.set_main_group(optGroup);

	try
	{
		if (!optContext.parse(argc, argv))
		{
			fprintf(stderr, "# ERROR: Cannot parse options.\n");
			return 1;
		}
	}
	catch (Glib::OptionError& error)
	{
		fprintf(stderr, "# ERROR: Cannot parse options.\n");
		return 1;
	}

	bool dds2lcm = false;
	bool lcm2dds = false;
	if (bridgeMode.compare("dds2lcm") == 0)
	{
		dds2lcm = true;
	}
	if (bridgeMode.compare("lcm2dds") == 0)
	{
		lcm2dds = true;
	}
	if (!dds2lcm && !lcm2dds)
	{
		fprintf(stderr, "# ERROR: Bridge mode is not valid. Please use either dds2lcm or lcm2dds\n");
		return 1;
	}

	signal(SIGINT, signalHandler);

	lcm_t* lcm = lcm_create("udpm://");
	if (!lcm)
	{
		fprintf(stderr, "# ERROR: Cannot create LCM instance.\n");
		exit(EXIT_FAILURE);
	}

	px::Middleware mw;
	mw.init(argc, argv);

	mavlink_message_t_subscription_t* imageLCMSub = 0;
	mavlink_message_t_subscription_t* mavlinkLCMSub = 0;
	if (lcm2dds)
	{
		// create instance of shared memory client for each possible camera configuration
		imageClientVec.resize(4);

		imageClientVec.at(0).init(true, PxSHM::CAMERA_FORWARD_LEFT);
		imageClientVec.at(1).init(true, PxSHM::CAMERA_FORWARD_LEFT, PxSHM::CAMERA_FORWARD_RIGHT);
		imageClientVec.at(2).init(true, PxSHM::CAMERA_DOWNWARD_LEFT);
		imageClientVec.at(3).init(true, PxSHM::CAMERA_DOWNWARD_LEFT, PxSHM::CAMERA_DOWNWARD_RIGHT);

		dds_image_message_t_initialize(&dds_image_msg);

		// subscribe to LCM messages
		imageLCMSub = mavlink_message_t_subscribe(lcm, "IMAGES", &imageLCMHandler, 0);
		mavlinkLCMSub = mavlink_message_t_subscribe(lcm, "MAVLINK", &mavlinkLCMHandler, 0);

		// advertise DDS topics
		px::ImageTopic::instance()->advertise();
		px::MavlinkTopic::instance()->advertise();
	}

	if (dds2lcm)
	{
		// create instance of shared memory server for each possible camera configuration
		imageServerVec.resize(4);

		imageServerVec.at(0).init(getSystemID(), PX_COMP_ID_CAMERA, lcm, PxSHM::CAMERA_FORWARD_LEFT);
		imageServerVec.at(1).init(getSystemID(), PX_COMP_ID_CAMERA, lcm, PxSHM::CAMERA_FORWARD_LEFT, PxSHM::CAMERA_FORWARD_RIGHT);
		imageServerVec.at(2).init(getSystemID(), PX_COMP_ID_CAMERA, lcm, PxSHM::CAMERA_DOWNWARD_LEFT);
		imageServerVec.at(3).init(getSystemID(), PX_COMP_ID_CAMERA, lcm, PxSHM::CAMERA_DOWNWARD_LEFT, PxSHM::CAMERA_DOWNWARD_RIGHT);

		// subscribe to DDS messages
		px::Handler handler;
		handler = px::Handler(sigc::ptr_fun(imageDDSHandler));
		px::ImageTopic::instance()->subscribe(handler, px::SUBSCRIBE_ALL);

		handler = px::Handler(sigc::bind(sigc::ptr_fun(mavlinkDDSHandler), lcm));
		px::MavlinkTopic::instance()->subscribe(handler, px::SUBSCRIBE_ALL);
	}

	while (!quit)
	{
		lcm_handle(lcm);
	}

	mw.shutdown();

	if (lcm2dds)
	{
		mavlink_message_t_unsubscribe(lcm, mavlinkLCMSub);

		dds_image_message_t_finalize(&dds_image_msg);
	}
	lcm_destroy(lcm);

	return 0;
}
