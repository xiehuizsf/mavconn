/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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
 *   @brief Process for capturing images from the camera and send it over MAVlink to the groundstation.
 *
 *   @author Fabian Brun <mavteam@pixhawk.ethz.ch>
 */

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <glib.h>
// BOOST include
#include <boost/program_options.hpp>
// OpenCV includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
// Latency Benchmarking
#include <time.h>
// mavconn core includes
#include <mavconn.h>
#include <interface/shared_mem/PxSharedMemClient.h>

namespace config = boost::program_options;
using namespace std;

int sysid, compid;
uint64_t camno;
bool silent, verbose, debug;

#define PACKET_PAYLOAD		252
bool captureImage = false;
uint8_t id = NULL;
uint8_t freq = NULL;
uint8_t jpg_quality = NULL;
bool acked = false;
uint64_t timelast = NULL;
uint64_t timenext = 1000000;

lcm_t* lcmImage;
lcm_t* lcmMavlink;
mavlink_message_t tmp;
mavlink_data_transmission_handshake_t req, ack;



/**
 * @brief Handle incoming MAVLink packets containing images
 */
static void image_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavlink_message_t* msg, void * user)
{
	// Pointer to shared memory data
	PxSharedMemClient* client = static_cast<PxSharedMemClient*>(user);

	// Temporary memory for raw camera image
	cv::Mat img( 480, 640, CV_8UC1 );

	// Check if there are images
	uint64_t camId = client->getCameraID(msg);
	if (camId != 0)
	{
		// Copy one image from shared buffer
		if (!client->sharedMemCopyImage(msg, img))
		{
			cout << "No image could be retrieved, even camera ID was set." << endl;
			exit(1);
		}

		// Check for valid jpg_quality in request and adjust if necessary
                if (jpg_quality < 1 || jpg_quality > 100)
		{
                        jpg_quality = 50;
		}
                // Check for valid freq in request and adjust if necessary
                if(freq < 1 || freq > 25)
                {
                        freq = 5; // arbitrarily chosen
                }
                timenext = client->getTimestamp(msg);

                // only prepare&send image, if enough time between images
                if(timenext >= timelast + (1000000/freq))
                {
                        timelast = timenext;

                        // Encode image as JPEG
                        vector<uint8_t> jpg; ///< container for JPEG image data
                        vector<int> p (2); ///< params for cv::imencode. Sets the JPEG quality.
                        p[0] = CV_IMWRITE_JPEG_QUALITY;
                        p[1] = jpg_quality;
                        cv::imencode(".jpg", img, jpg, p);

                        // Prepare and send acknowledgment packet
                        //ack.state = 1;
                        ack.type = static_cast<uint8_t>( DATA_STREAM_TYPE_IMG_JPEG );
                        //ack.id = id;
                        ack.size = static_cast<uint32_t>( jpg.size() );
                        ack.packets = static_cast<uint8_t>( ack.size/PACKET_PAYLOAD );
                        if (ack.size % PACKET_PAYLOAD) { ++ack.packets; } // one more packet with the rest of data
                        ack.payload = static_cast<uint8_t>( PACKET_PAYLOAD );
                        //ack.freq = freq;
                        ack.jpg_quality = jpg_quality;

                        mavlink_msg_data_transmission_handshake_encode(sysid, compid, &tmp, &ack);
                        mavlink_message_t_publish(lcmMavlink, MAVLINK_MAIN, &tmp);

                        // Send image data (split up into smaller chunks first, then sent over MAVLink)
                        uint8_t data[PACKET_PAYLOAD];
                        uint16_t byteIndex = 0;
                        if (verbose) printf("there are %02d packets waiting to be sent (%05d bytes). start sending...\n", ack.packets, ack.size);

                        for (uint8_t i = 0; i < ack.packets; ++i)
                        {
                                // Copy PACKET_PAYLOAD bytes of image data to send buffer
                                for (uint8_t j = 0; j < PACKET_PAYLOAD; ++j)
                                {
                                        if (byteIndex < ack.size)
                                        {
                                                data[j] = (uint8_t)jpg[byteIndex];
                                        }
                                        // fill packet data with padding bits
                                        else
                                        {
                                                data[j] = 0;
                                        }
                                        ++byteIndex;
                                }
                                // Send ENCAPSULATED_IMAGE packet
                                mavlink_msg_encapsulated_data_pack(sysid, compid, &tmp, i, ack.id, data);
                                mavlink_message_t_publish(lcmMavlink, MAVLINK_MAIN, &tmp);
                                if (verbose) printf("sent packet %02d successfully\n", i+1);
                        }
                }

	}
}

/**
 * @brief Handle incoming MAVLink packets containing ACTION messages
 */
static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavlink_message_t* msg, void * user)
{
        if (msg->msgid == MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE)
	{
                mavlink_msg_data_transmission_handshake_decode(msg, &req);

                if (/*req.state == 0 &&*/ req.type == DATA_STREAM_TYPE_IMG_JPEG)
		{
                        // copy request data
                        //freq = req.freq;
                        jpg_quality = req.jpg_quality;
                        // generate ID - for now, just use the data type as an ID
                        id = DATA_STREAM_TYPE_IMG_JPEG;

			// start recording image data
			captureImage = true;
		}
		/*else if (req.state == -1)
		{
				captureImage = false;
		}*/
	}
}


void* lcm_wait(void* lcm_ptr)
{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (true)
	{
		lcm_handle (lcm);
	}
	return NULL;
}

/*
 * @brief Main: registers at mavlink channel for images; starts image handler
 */
int main(int argc, char* argv[])
{
	// ----- Handling Program options
	config::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("sysid,a", config::value<int>(&sysid)->default_value(42), "ID of this system, 1-256")
		("compid,c", config::value<int>(&compid)->default_value(30), "ID of this component")
		("camno,c", config::value<uint64_t>(&camno)->default_value(0), "ID of the camera to read")
		("silent,s", config::bool_switch(&silent)->default_value(false), "suppress outputs")
		("verbose,v", config::bool_switch(&verbose)->default_value(false), "verbose output")
		("debug,d", config::bool_switch(&debug)->default_value(false), "Emit debug information")
		;
	config::variables_map vm;
	config::store(config::parse_command_line(argc, argv, desc), vm);
	config::notify(vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	// ----- Setting up communication and data for images
	// Creating LCM network provider
	lcmImage = lcm_create ("udpm://");
	lcmMavlink = lcm_create ("udpm://");
	if (!lcmImage || !lcmMavlink)
		exit(EXIT_FAILURE);

	PxSharedMemClient* cam = new PxSharedMemClient();
	mavlink_message_t_subscription_t * img_sub  = mavlink_message_t_subscribe (lcmImage, "IMAGES", &image_handler, cam);
	mavlink_message_t_subscription_t * comm_sub = mavlink_message_t_subscribe (lcmMavlink, "MAVLINK", &mavlink_handler, lcmMavlink);

	cout << "MAVLINK client ready, waiting for data..." << endl;

	// Creating thread for MAVLink handling
	GThread* lcm_mavlinkThread;
	GError* err;

	if( !g_thread_supported() ) {
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	// Start thread for handling messages on the MAVLINK channel
	cout << "Starting thread for mavlink handling..." << endl;
	if( (lcm_mavlinkThread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcmMavlink, TRUE, &err)) == NULL)
	{
		cout << "Thread create failed: " << err->message << "!!" << endl;
		g_error_free(err) ;
	}
	cout << "MAVLINK client ready, waiting for requests..." << endl;

	// Start the image transmission
	while (true)
	{
		if (captureImage)
		{
			// blocking wait for image channel
			lcm_handle(lcmImage);
		}
	}

	// Clean up
	cout << "Stopping thread for image capture..." << endl;
	g_thread_join(lcm_mavlinkThread);

	cout << "Everything done successfully - Exiting" << endl;

	mavlink_message_t_unsubscribe (lcmImage, img_sub);
	mavlink_message_t_unsubscribe (lcmMavlink, comm_sub);
	lcm_destroy (lcmImage);
	lcm_destroy (lcmMavlink);
	delete cam;

	exit(EXIT_SUCCESS);
}
