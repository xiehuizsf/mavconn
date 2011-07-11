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
 *   @brief Process for capturing images from the camera, preparing and feeding them to ffmpeg to stream video over the network.
 *
 *   @author Fabian Brun <mavteam@pixhawk.ethz.ch>
 */

#include <inttypes.h>
#include <cstdlib>
// BOOST include
#include <boost/program_options.hpp>
// mavconn core includes
#include <mavconn.h>
#include <interface/shared_mem/PxSharedMemClient.h>

using namespace std;

namespace config = boost::program_options;
bool silent, verbose, debug;
int sysid, compid;
uint64_t camno;

bool startStreaming = false;

lcm_t* lcmMavlink;
mavlink_message_t tmp;
mavlink_video_stream_t req, ack;


/**
 * @brief Handle incoming MAVLink message packets
 */
static void mavlink_handler(const lcm_recv_buf_t *rbuf, const char * channel, const mavlink_message_t* msg, void * user)
{
    if (verbose) cout << "0" << endl;
    if (msg->msgid == MAVLINK_MSG_ID_VIDEO_STREAM)
    {
        mavlink_msg_video_stream_decode(msg, &req);
        // check if we are the targeted system
        if (req.target == sysid)
        {
            // send ACK back: use same message, just change target
            ack.target = static_cast<uint8_t>(msg->sysid);
            ack.start_stop = static_cast<uint8_t>(req.start_stop);

            // start video streaming
            if(req.start_stop)
            {
                if (verbose) cout << "starting videostream..." << endl;
                system("/bin/sh ~/px_videostreamer.sh &");
                if (verbose) cout << "started..." << endl;
            }
            // stop video streaming
            else
            {
                if (verbose) cout << "stopping videostream..." << endl;
                //system("kill `cat ~/videostreamer.pid`");
                system("kill `pidof px_videoprepare`");
            }
            if (verbose) cout << "1" << endl;
            mavlink_msg_video_stream_encode(sysid, compid, &tmp, &ack);
            if (verbose) cout << "2" << endl;
            mavlink_message_t_publish(lcmMavlink, MAVLINK_MAIN, &tmp);
            if (verbose) cout << "3" << endl;
        }
    }
}

/**
 * @brief Main: registers at mavlink channel, calls mavlink_handler
 */
int main(int argc, char* argv[])
{
    // handling Program options
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

    // creating LCM network provider
    lcmMavlink = lcm_create(NULL); //"udpm://");
    if (!lcmMavlink)
    {
        exit(EXIT_FAILURE);
    }

    PxSharedMemClient* cam = new PxSharedMemClient();
    mavlink_message_t_subscription_t * comm_sub = mavlink_message_t_subscribe(lcmMavlink, "MAVLINK", &mavlink_handler, lcmMavlink);

    while (true)
    {
        // blocking wait for mavlink channel
        lcm_handle(lcmMavlink);
        if (verbose) cout << "4" << endl;
    }
    if (verbose) cout << "5" << endl;

    // clean up
    mavlink_message_t_unsubscribe(lcmMavlink, comm_sub);
    lcm_destroy(lcmMavlink);
    delete cam;
    exit(EXIT_SUCCESS);
}
