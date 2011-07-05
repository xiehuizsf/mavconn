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

#include <inttypes.h>
#include <cstdlib>
#include <glib.h>
// BOOST include
#include <boost/program_options.hpp>
// OpenCV includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
// Latency Benchmarking
//#include <time.h>
// mavconn core includes
#include <mavconn.h>
#include <interface/shared_mem/PxSharedMemClient.h>
#include <interface/shared_mem/PxSHMImageClient.h>

#ifndef max
    #define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef min
    #define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif

using namespace std;

namespace config = boost::program_options;
bool silent, verbose, debug;
int sysid, compid;
uint64_t camno;

#define PACKET_PAYLOAD		252
bool startCapture = false;
bool captureSingleImage = false;

//uint8_t id = 0;
//uint8_t freq = 0;
//uint8_t jpg_quality = 0;
//bool acked = false;
bool ackDataSet = false;

lcm_t* lcmImage;
lcm_t* lcmMavlink;
mavlink_message_t tmp;
mavlink_data_transmission_handshake_t ack;


/**
 * @brief Handle incoming MAVLink packets containing images
 */
static void image_handler(const lcm_recv_buf_t *rbuf, const char * channel, const mavlink_message_t* msg, void * user)
{
    uint64_t timelast = 0;
    uint64_t timenext = 1000000;

    // Pointer to shared memory data
    PxSharedMemClient* client = static_cast<PxSharedMemClient*>(user);
    // new interface, TODO FIXME does not work yet
    //PxSHMImageClient client;
    //if (!client.init(false, PxSHM::CAMERA_FORWARD_RIGHT))
    //{
    //    cout << "something went wrong..." << endl;
    //    exit(1);
    //}

    // Temporary memory for raw camera image
    cv::Mat img( 480, 640, CV_8UC1 );

    // Check if there are images
    //uint64_t camId = client->getCameraID(msg);
    //if (camId != 0)
    //{

    // Copy one image from shared buffer
    if (!client->sharedMemCopyImage(msg, img))
    {
        cout << "No image could be retrieved, exiting." << endl;
        exit(1);
    }
    // new interface
    //if (!client.readMonoImage(msg, img))
    //{
    //    cout << "No image could be retrieved, exiting." << endl;
    //    exit(1);
    //}

    timenext = client->getTimestamp(msg);
    //timenext = client.getTimestamp(msg);

    // only prepare and send an image, if there is enough time between images
    if (timenext >= timelast + (1000000/ack.freq))
    {
        if (verbose) cout << "An image is ready! Preparing to send... ";

        timelast = timenext;

        // Encode image as JPEG
        vector<uint8_t> jpg; ///< container for JPEG image data
        vector<int> p (2); ///< params for cv::imencode. Sets the JPEG quality.
        p[0] = CV_IMWRITE_JPEG_QUALITY;
        p[1] = ack.jpg_quality;
        cv::imencode(".jpg", img, jpg, p);

        // Prepare and send acknowledgment packet
        ack.size = static_cast<uint32_t>( jpg.size() );
        ack.packets = static_cast<uint8_t>( ack.size/PACKET_PAYLOAD );
        if (ack.size % PACKET_PAYLOAD) { ++ack.packets; } // one more packet with the rest of data
        ack.payload = static_cast<uint8_t>( PACKET_PAYLOAD );
        ackDataSet = true;

        // Send image data (split up into smaller chunks first, then sent over MAVLink)
        uint8_t data[PACKET_PAYLOAD];
        uint16_t byteIndex = 0;
        if (verbose) printf("sending... (%02d packets, %05d bytes)\n", ack.packets, ack.size);

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
            //if (verbose) printf("sent packet %02d successfully\n", i+1);
        }
    }
    //}

}


/**
 * @brief Handle incoming MAVLink packets containing ACTION messages
 */
static void mavlink_handler(const lcm_recv_buf_t *rbuf, const char * channel, const mavlink_message_t* msg, void * user)
{
    if (msg->msgid == MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE)
    {
        mavlink_data_transmission_handshake_t req;
        mavlink_msg_data_transmission_handshake_decode(msg, &req);

        if (verbose) cout << "msg (transmission_handshake): " << (int)req.target << " / " << (int)req.state << " / " << (int)req.type << " " << (int)MAVLINK_DATA_STREAM_IMG_JPEG << endl;
        //if (verbose) cout << "other approach: " << req->get

        if (req.target == sysid && !req.state && req.type == MAVLINK_DATA_STREAM_IMG_JPEG)
        {
            cout << "A handshake message arrived!" << endl;

            // prepare ACK packet
            ack.target = msg->sysid;
            ack.state = !req.state;
            ack.id = MAVLINK_DATA_STREAM_IMG_JPEG; // TODO create real IDs
            ack.type = MAVLINK_DATA_STREAM_IMG_JPEG;
            ack.size = 0;
            ack.packets = 0;
            ack.payload = 0;
            // Check for valid freq in request and adjust if necessary (min: -1fps, max:30fps)
            ack.freq = max(min(req.freq, 30), -1);
            // Check for valid jpg_quality in request and adjust if necessary
            ack.jpg_quality = req.jpg_quality;
            if (ack.jpg_quality < 1 || ack.jpg_quality > 100)
            {
                ack.jpg_quality = 50;
            }

            // start/stop image stream
            // frequency of 0 means "take a single picture
            if (ack.freq)
            {
                startCapture = true;
                if (verbose) cout << "It is a request to start the stream: starting..." << endl;
            }
            if (ack.freq == 0)
            {
                captureSingleImage = true;
                if (verbose) cout << "It is a request to capture a single image: capturing..." << endl;
            }
            if (ack.freq < 0)
            {
                startCapture = false;
                if (verbose) cout << "It is a request to stop the stream: stopping..." << endl;
            }

            // wait for image handler to set the data for the ack packet
            if (verbose) cout << "Waiting for the image handler do calculate sizing data..." << endl;
            while(!ackDataSet)
            {
                // do nothing
                usleep(1);
            }

            // ack message was set, can now be sent
            if (verbose) cout << "Sending back an ACK message..." << endl;
            mavlink_msg_data_transmission_handshake_encode(sysid, compid, &tmp, &ack);
            mavlink_message_t_publish(lcmMavlink, MAVLINK_MAIN, &tmp);
        }
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
    // handling program options
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

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    // setting up lcm communication
    lcmImage   = lcm_create(NULL); //"udpm://");
    lcmMavlink = lcm_create(NULL); //"udpm://");
    if (!lcmImage || !lcmMavlink)
    {
        exit(EXIT_FAILURE);
    }

    PxSharedMemClient* cam = new PxSharedMemClient();
    mavlink_message_t_subscription_t * img_sub  = mavlink_message_t_subscribe (lcmImage,   "IMAGES",  &image_handler,   cam);
    mavlink_message_t_subscription_t * comm_sub = mavlink_message_t_subscribe (lcmMavlink, "MAVLINK", &mavlink_handler, lcmMavlink);
    //if (verbose) cout << "MAVLINK client ready, waiting for data..." << endl;

    // Creating thread for MAVLink handling
    GThread* lcm_mavlinkThread;
    GError* err;
    if (!g_thread_supported())
    {
        g_thread_init(NULL);
        // Only initialize g thread if not already done
    }

    // start thread for handling messages on the MAVLINK channel
    //if (verbose) cout << "Starting thread for mavlink handling..." << endl;
    if ( !(lcm_mavlinkThread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcmMavlink, TRUE, &err)) )
    {
        cout << "GThread creation failed: " << err->message << endl;
        g_error_free(err);
    }
    if (verbose) cout << "MAVLINK client ready, waiting for requests..." << endl;

    if (debug)
    {
        mavlink_message_t testmsg;
        mavlink_data_transmission_handshake_t test;

        test.target = sysid;
        test.state = 0;
        test.id = MAVLINK_DATA_STREAM_IMG_JPEG;
        test.type = MAVLINK_DATA_STREAM_IMG_JPEG;
        test.size = 0;
        test.packets = 0;
        test.payload = 0;
        test.freq = 15;
        test.jpg_quality = 50;

        mavlink_msg_data_transmission_handshake_encode(sysid, compid, &testmsg, &test);
        mavlink_message_t_publish(lcmMavlink, MAVLINK_MAIN, &testmsg);
        sleep(1);
    }

    // start the image transmission
    while (true)
    {
        //if (debug) cout << (int)startCapture << " / " << (int)captureSingleImage << endl;
        if (startCapture || captureSingleImage)
        {
            // blocking wait for image channel
            lcm_handle(lcmImage);
        }
        captureSingleImage = false;
    }

    // clean up
    //if (verbose) cout << "Stopping thread for image capture..." << endl;
    g_thread_join(lcm_mavlinkThread);

    cout << "Everything done successfully - Exiting" << endl;

    mavlink_message_t_unsubscribe (lcmImage, img_sub);
    mavlink_message_t_unsubscribe (lcmMavlink, comm_sub);
    lcm_destroy (lcmImage);
    lcm_destroy (lcmMavlink);
    delete cam;

    exit(EXIT_SUCCESS);
}
