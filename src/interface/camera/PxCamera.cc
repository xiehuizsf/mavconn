#include "PxCamera.h"

#include <cstdio>

PxCamera::PxCamera()
 : verbose(false)
 , mGrabThread(0)
 , mTerminateGrabThread(false)
{

}

PxCamera::~PxCamera()
{

}
