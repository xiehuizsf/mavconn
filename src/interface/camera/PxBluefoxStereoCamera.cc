#include "PxBluefoxStereoCamera.h"

#include <sys/time.h>

PxBluefoxStereoCamera::PxBluefoxStereoCamera(mvIMPACT::acquire::Device* _cameraLeft,
											 mvIMPACT::acquire::Device* _cameraRight)
 : lastSequenceNum(0)
 , maxRequests(4)
{
	cameraLeft = std::tr1::shared_ptr<PxBluefoxCamera>(new PxBluefoxCamera(_cameraLeft));
	cameraRight = std::tr1::shared_ptr<PxBluefoxCamera>(new PxBluefoxCamera(_cameraRight));
}

PxBluefoxStereoCamera::~PxBluefoxStereoCamera()
{

}

bool
PxBluefoxStereoCamera::init(void)
{
	if (!cameraLeft->init())
	{
		return false;
	}
	if (!cameraRight->init())
	{
		return false;
	}

	return true;
}

void
PxBluefoxStereoCamera::destroy(void)
{
	cameraLeft->destroy();
	cameraRight->destroy();
}

bool
PxBluefoxStereoCamera::setConfig(const PxCameraConfig& config)
{
	frameRate = config.getFrameRate();
	timeout_ms = 1.0f / frameRate * 10000.0f; //10 sec hard-time out limit (with this it is ensured that all threads can be shut off, but no trigger interval can be larger than this!)

	mode = config.getMode();

	if (!cameraLeft->setConfig(config, true))
	{
		return false;
	}
	if (!cameraRight->setConfig(config, false))
	{
		return false;
	}

	// TODO: Remove this if statement once ExposeSet issue is resolved
	// manually set exposure for right camera based on those with
	// which latest image from left camera was taken
	if (mode == PxCameraConfig::AUTO_MODE &&
	   !cameraRight->setMode(PxCameraConfig::MANUAL_MODE))
	{
		return false;
	}
	
	return true;
}

bool
PxBluefoxStereoCamera::start(void)
{
	exitImageThreads = false;
	imageAvailableCond.reset(new Glib::Cond);
	imageAvailable = 0;

	mvIMPACT::acquire::SystemSettings systemSettingsL(cameraLeft->dev);
	systemSettingsL.requestCount.write(maxRequests);
	mvIMPACT::acquire::SystemSettings systemSettingsR(cameraRight->dev);
	systemSettingsR.requestCount.write(maxRequests);

	try
	{
		stereoImageThread = Glib::Thread::create(sigc::mem_fun(this, &PxBluefoxStereoCamera::stereoImageHandler), true);
	}
	catch (const Glib::ThreadError& e)
	{
		fprintf(stderr, "# ERROR: Cannot create image handling thread.\n");
		return false;
	}

	return true;
}

bool
PxBluefoxStereoCamera::stop(void)
{
	exitImageThreads = true;

	stereoImageThread->join();

	return true;
}

bool
PxBluefoxStereoCamera::grabFrame(cv::Mat& imageLeft, cv::Mat& imageRight,
								 uint32_t& skippedFrames, uint32_t& sequenceNum)
{
	imageMutex.lock();
	if (imageAvailable != ALL_IMAGES_AVAILABLE && !exitImageThreads)
	{
		do
		{
			imageAvailableCond->wait(imageMutex);
		}
		while (imageAvailable != ALL_IMAGES_AVAILABLE && !exitImageThreads);
	}

	if (exitImageThreads){ imageMutex.unlock(); return false; }

	cameraLeft->image.copyTo(imageLeft);
	cameraRight->image.copyTo(imageRight);
	sequenceNum = cameraLeft->imageSequenceNr;

	imageAvailable = 0;
	imageMutex.unlock();

	if (sequenceNum > lastSequenceNum)
	{
		skippedFrames = sequenceNum - lastSequenceNum - 1;
		lastSequenceNum = sequenceNum;
	}
	else
	{
		// sequence numbers wrong!
		return false;
	}

	return true;
}

void
PxBluefoxStereoCamera::stereoImageHandler(void)
{
	for (int i = 0; i < maxRequests; ++i)
	{
		cameraLeft->functionInterface->imageRequestSingle();
		cameraRight->functionInterface->imageRequestSingle();
	}

	int requestNrL = cameraLeft->functionInterface->imageRequestWaitFor(9000);
	int requestNrR = cameraRight->functionInterface->imageRequestWaitFor(1000);

	while (!exitImageThreads)
	{
		ssize_t seqL = 666, seqR = 666;
		bool requestFailed = false;

		const mvIMPACT::acquire::Request *requestL, *requestR;

		if (cameraLeft->functionInterface->isRequestNrValid(requestNrL))
		{
			requestL = cameraLeft->functionInterface->getRequest(requestNrL);
			if (requestL->isOK())
			{
				seqL = requestL->infoFrameNr.read();
				if (cameraRight->functionInterface->isRequestNrValid(requestNrR))
				{
					requestR = cameraRight->functionInterface->getRequest(requestNrR);
					if (requestR->isOK())
					{
						seqR = requestR->infoFrameNr.read();
					}
					else
					{
						requestFailed = true;
					}
				}
				else
				{
					requestFailed = true;
					cameraLeft->functionInterface->imageRequestUnlock(requestNrL);	//we had a correct left request give it back
					cameraLeft->functionInterface->imageRequestSingle();
				}
			}
			else
			{
				requestFailed = true;
			}
		}
		else
		{
			requestFailed = true;
		}

		//printf("PreSync: L: %llu\t R: %llu\treq: %d\n", seqL, seqR, requestFailed);

		while (!requestFailed && seqL > seqR)
		{
			cameraRight->functionInterface->imageRequestUnlock(requestNrR);
			cameraRight->functionInterface->imageRequestSingle();
			requestNrR = cameraRight->functionInterface->imageRequestWaitFor(timeout_ms);
			if (cameraLeft->functionInterface->isRequestNrValid(requestNrR))
			{
				requestR = cameraLeft->functionInterface->getRequest(requestNrR);
				if (requestR->isOK())
				{
					seqR = requestR->infoFrameNr.read();
				}
			}
			else
			{
				requestFailed = true;
				cameraLeft->functionInterface->imageRequestUnlock(requestNrL);	//we had a correct left request give it back
				cameraLeft->functionInterface->imageRequestSingle();
			}
		}

		while (!requestFailed && seqL < seqR)
		{
			cameraLeft->functionInterface->imageRequestUnlock(requestNrL);
			cameraLeft->functionInterface->imageRequestSingle();
			requestNrL = cameraLeft->functionInterface->imageRequestWaitFor(timeout_ms);
			if (cameraLeft->functionInterface->isRequestNrValid(requestNrL))
			{
				requestL = cameraLeft->functionInterface->getRequest(requestNrL);
				if (requestL->isOK())
				{
					seqL = requestL->infoFrameNr.read();
				}
			}
			else
			{
				requestFailed = true;
				cameraRight->functionInterface->imageRequestUnlock(requestNrR);	//we had a correct right request give it back
				cameraRight->functionInterface->imageRequestSingle();
			}
		}

		//printf("         L: %llu\t R: %llu\n", seqL, seqR);

		if (!requestFailed)
		{
			//at this point in time we have both frames with equal sequence number, store them
			imageMutex.lock();
			cameraLeft->convertToCvMat(requestL, cameraLeft->image);
			cameraRight->convertToCvMat(requestR, cameraRight->image);
			cameraLeft->imageSequenceNr = seqL;
			cameraRight->imageSequenceNr = seqR;

			// TODO: Remove this if statement once ExposeSet issue is resolved
			if (mode == PxCameraConfig::AUTO_MODE)
			{
				// set exposure of right camera based on image from left camera
				cameraRight->setExposureTime(requestL->infoExposeTime_us.read());
			}

			imageAvailable = ALL_IMAGES_AVAILABLE;
			imageAvailableCond->signal();
			imageMutex.unlock();

			//return the request and queue up for the next one
			cameraLeft->functionInterface->imageRequestUnlock(requestNrL);
			cameraLeft->functionInterface->imageRequestSingle();
			cameraRight->functionInterface->imageRequestUnlock(requestNrR);
			cameraRight->functionInterface->imageRequestSingle();

			Glib::Thread::yield();

			requestNrL = cameraLeft->functionInterface->imageRequestWaitFor(timeout_ms);
			requestNrR = cameraRight->functionInterface->imageRequestWaitFor(timeout_ms);
		}
		else
		{
			imageMutex.lock();
			exitImageThreads = true;		//we want to stop everything now...
			imageAvailableCond->signal();
			imageMutex.unlock();
		}
	}

	cameraLeft->functionInterface->imageRequestReset(0, 0);
	cameraRight->functionInterface->imageRequestReset(0, 0);
}
