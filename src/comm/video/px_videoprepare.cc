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
 *   @brief Process for preparing camera images for further use by ffmpeg.
 *
 *   @author Fabian Brun <mavteam@pixhawk.ethz.ch>
 */

#include <inttypes.h>
#include <cstdlib>
// OpenCV include
#include <opencv/highgui.h>
// mavconn core includes
#include <mavconn.h>
#include <interface/shared_mem/PxSharedMemClient.h>

using namespace std;
lcm_t* lcmImage;


/**
 * @brief Handle incoming MAVLink image packets
 */
static void image_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavlink_message_t* msg, void * user)
{
    // Pointer to shared memory data
    PxSharedMemClient* client = static_cast<PxSharedMemClient*>(user);

    // TODO remove safely
    //mavlink_image_available_t image;
    //mavlink_msg_image_available_decode(msg, &image);
    // check if there are images
    //uint64_t camId = client->getCameraID(msg);
    //if (camId != 0)
    //{
    // copy one image from shared buffer
    IplImage* img = cvCreateImage(cvSize(640, 480),IPL_DEPTH_8U, 1);
    if (!client->sharedMemCopyImage(msg, img))
    {
        exit(EXIT_FAILURE);
    }

    // write to stdout, but first flush the stream
    fflush(stdout);
    write(1, img->imageData, img->imageSize);
    // fill unused (Y)UV channels
    for (int i=0; i < img->imageSize/2; i++)
    {
        putchar(0x80);
    }
    //}
}

/**
 * @brief Main: registers at image channel, calls image_handler
 */
int main(int argc, char* argv[])
{
    // creating LCM network provider
    lcm_t* lcmImage = lcm_create(NULL);
    if (!lcmImage)
    {
        exit(EXIT_FAILURE);
    }
    PxSharedMemClient* cam = new PxSharedMemClient();
    mavlink_message_t_subscription_t * img_sub = mavlink_message_t_subscribe (lcmImage, "IMAGES", &image_handler, cam);

    while (true)
    {
        // blocking wait for image channel
        lcm_handle(lcmImage);
    }

    // clean up
    mavlink_message_t_unsubscribe(lcmImage, img_sub);
    lcm_destroy(lcmImage);
    delete cam;
    exit(EXIT_SUCCESS);
}
