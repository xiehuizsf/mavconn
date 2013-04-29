/**
   @author Hui Xie <xie1@ualberta.ca>
   \author Bryan Godbolt <godbolt@ece.ualberta.ca>
   @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
   @author Aakash Vasudevan <avasudev@ualberta.ca>
   \author Hasitha Senanayake <senanaya@ualberta.ca>

//   \mainpage
//   This project contains the code for the autopilot system.  This documentation is available on the
//   local network at http://doc/autopilot/html/index.html .
//   The central version should be used (as opposed to a locally generated version) since it is regenerated
//   every time a push is made to the central repository.

//   \p The contents of the root folder are
//   - \b doc/ project documentation
//   - \b README describes folder structure
//   - \b src/ source code
//   - \b utils/ sample programs to demonstrate various aspects of the system (e.g., counter board i/o)
//   - \b extern/ external libraries used to build the autopilot

//   @note all debugging messages should be printed using the ::debug() function.  See the Debug class.
 */
#include<iostream>
/* Boost Headers */
#include <boost/thread.hpp>

/* Project Headers */
#include "MainApp.h"
/* Test: Later can be deleted */
#include "Debug.h"
#include "pid_channel.h"
#include "attitude_pid.h"
using namespace std;
int main()
{
	cout<<"Welcome to Mavconn-autopilot"<<endl;
	debug()<<"Just try to debug";
	pid_channel roll;
	attitude_pid orientation;
//	message()<<roll;
}
