#include "PxFireflyCameraManager.h"

#include "PxFireflyCamera.h"
#include "PxFireflyStereoCamera.h"

PxFireflyCameraManager::PxFireflyCameraManager()
{
	context = dc1394_new();
}

PxFireflyCameraManager::~PxFireflyCameraManager()
{
	dc1394_free(context);
}

PxCameraPtr
PxFireflyCameraManager::generateCamera(uint64_t serialNum)
{
	if (!context)
	{
		fprintf(stderr, "# ERROR: dc1394 context is invalid.\n");
		return PxCameraPtr();
	}

	dc1394camera_t* camera = getCamera(serialNum);
	if (!camera)
	{
		return PxCameraPtr();
	}
	else
	{
		return PxCameraPtr(new PxFireflyCamera(camera));
	}
}

PxStereoCameraPtr
PxFireflyCameraManager::generateStereoCamera(uint64_t serialNum1, uint64_t serialNum2)
{
	if (!context)
	{
		fprintf(stderr, "# ERROR: dc1394 context is invalid.\n");
		return PxStereoCameraPtr();
	}

	return PxStereoCameraPtr(
			new PxFireflyStereoCamera(getCamera(serialNum1),
									  getCamera(serialNum2)));
}

int
PxFireflyCameraManager::getCameraCount(void) const
{
	if (!context)
	{
		fprintf(stderr, "# ERROR: dc1394 context is invalid.\n");
		return 0;
	}

	// get all connected cameras
	dc1394camera_list_t* camList;
	dc1394error_t error = dc1394_camera_enumerate(context, &camList);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Error during enumeration: %s\n", dc1394_error_get_string(error));
		return 0;
	}

	int cameraCount = camList->num;

	dc1394_camera_free_list(camList);

	return cameraCount;
}

dc1394camera_t*
PxFireflyCameraManager::getCamera(uint64_t serialNum)
{
	if (context == NULL)
	{
		fprintf(stderr, "# ERROR: dc1394 context is invalid.\n");
		return 0;
	}

	//get all connected cameras
	dc1394camera_list_t* camList;
	dc1394error_t error = dc1394_camera_enumerate(context, &camList);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Error during enumeration: %s\n", dc1394_error_get_string(error));
		return 0;
	}

	dc1394camera_t* camera = 0;

	// Open camera
	for (unsigned int i = 0; i < camList->num; i++)
	{
		// the lower 32-bit half of camera->guid represents the printed serial on the camera
		if ((camList->ids[i].guid & 0xFFFFFFFFLL) == serialNum)
		{
			// this camera's serial nr matches the requested one
			// set the capture structure accordingly
			camera = dc1394_camera_new(context, camList->ids[i].guid);

			if (!camera)
			{
				fprintf(stderr, "# ERROR: dc1394_camera could not be created.\n");
				return 0;
			}

			// we found a camera, stop iterating here
			break;
		}
	}
	dc1394_camera_free_list(camList);

	if (!camera)
	{
		fprintf(stderr, "# ERROR: Cannot find Firefly camera with "
						"serial number %lu.\n", serialNum);
	}

	return camera;
}
