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
 *   @brief Definition of libdc1394v2 based camera interface
 *
 *   @author Pascal Dufour <mavteam@student.ethz.ch>
 *
 */

#include <inttypes.h>
#include <dc1394.h>
#include <opencv2/core/core.hpp>

typedef enum {
	CAMERA_SUCCESS = 0,
	CAMERA_FAILURE = -1,
	CAMERA_SYNCHRONIZATION_FAILURE = -2
}PxCameraError;

typedef enum {
	CAMERA_MANUAL_MODE = 0,
	CAMERA_AUTO_MODE = 1
} PxCameraMode;

class PxCameraSettings
{
public:
	PxCameraSettings(PxCameraMode _mode = CAMERA_AUTO_MODE, uint32_t _shutter = 2000, uint32_t _gain = 120, uint32_t _gamma = 0, uint32_t _brightness = 2047)
	 : mode(_mode)
	 , shutter(_shutter)
	 , gain(_gain)
	 , gamma(_gamma)
	 , brightness(_brightness)
	{

	}

	PxCameraMode getMode(void) const { return mode; }
	uint32_t getShutter(void) const { return shutter; }
	uint32_t getGain(void) const { return gain; }
	uint32_t getGamma(void) const { return gamma; }
	uint32_t getBrightness(void) const { return brightness; }

private:
	PxCameraMode mode;
	uint32_t shutter;
	uint32_t gain;
	uint32_t gamma;
	uint32_t brightness;
};

class PxCameraCapture
{
public:
	PxCameraError init(uint32_t serialNum, bool special = false);
	PxCameraError start(void);
	PxCameraError destroy(void);

	PxCameraError grabFrame(cv::Mat& image, uint32_t *skipped_frames = NULL, uint32_t *image_sequence = NULL);

	PxCameraError setSettings(PxCameraSettings& settings);
	dc1394error_t setFrameRate(float frameRate);

	PxCameraError setExternalTrigger(void);
	PxCameraError setStrobe(uint32_t pin);
	PxCameraError startTransmission(void);

	uint64_t serialNum; // serial number of camera

	unsigned int busIdx;
	dc1394camera_t* camera; //camera connected to this capture structure
	dc1394_t* context;
	bool special;

private:
	PxCameraError setMode(PxCameraMode mode);
	PxCameraError setShutter(uint32_t shutter);
	PxCameraError setGain(uint32_t gain);
	PxCameraError setGamma(uint32_t gamma);
	PxCameraError setBrightness(uint32_t brightness);

	PxCameraError getGPIOPinDirection(uint32_t pin, uint32_t& pDirection);
	PxCameraError setGPIOPinDirection(uint32_t pin, uint32_t pDirection);

	PxCameraError isStrobeEnabled(uint32_t pin, bool& flag);
	PxCameraError isStrobeConfigurable(uint32_t pin, bool& flag);
	PxCameraError setStrobeSource(uint32_t pin);
};

// new stereo interface
PxCameraError PxInitializeStereoCapture(PxCameraCapture &capture1, PxCameraCapture &capture2, uint32_t serialNr1, uint32_t serialNr2, bool extTrigger = false, bool automode = false, uint32_t exposure = 2000, uint32_t gain = 120, uint32_t gamma = 0);
PxCameraError PxGrabStereoFrames(PxCameraCapture &capture1, PxCameraCapture &capture2, cv::Mat& image1, cv::Mat& image2, uint32_t *skipped_frames = NULL, uint32_t *image_sequence = NULL, bool extTrigger = false);
