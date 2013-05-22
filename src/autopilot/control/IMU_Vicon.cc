/*******************************************************************************
 * Copyright 2013 Hui Xie
 * 
 * This file is part of ANCL Autopilot.
 * 
 *     ANCL Autopilot is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 * 
 *     ANCL Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 * 
 *     You should have received a copy of the GNU General Public License
 *     along with ANCL Autopilot.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

#include "IMU_Vicon.h"
#include "LogFile.h"
#include <iostream>
//IMU_thread(this)

lcm_t* IMU_Vicon::lcm = lcm_create ("udpm://");
mavconn_mavlink_msg_container_t_subscription_t* IMU_Vicon::comm_sub;

IMU_Vicon::IMU_Vicon():IMU_thread(IMU_read(this))
{
//	lcm = lcm_create ("udpm:");	
	comm_sub = mavconn_mavlink_msg_container_t_subscribe (lcm, MAVLINK_MAIN, &IMU_Vicon::mavlink_handler, NULL);
	MainApp::add_thread(&IMU_thread, std::string("IMU Vicon"));
}

IMU_Vicon::~IMU_Vicon()
{
	//TODO
	mavconn_mavlink_msg_container_t_unsubscribe(lcm, comm_sub);
}

IMU_Vicon* IMU_Vicon::_instance;
boost::mutex IMU_Vicon::_instance_lock;

IMU_Vicon* IMU_Vicon::getInstance()
{
	boost::mutex::scoped_lock lock(_instance_lock);
	if (!_instance)
		_instance = new IMU_Vicon;
	return _instance;
}




blas::vector<double> IMU_Vicon::viconPosition(3);
blas::vector<double> IMU_Vicon::viconVelocity(3);
blas::vector<double> IMU_Vicon::viconAngle(3);
blas::vector<double> IMU_Vicon::viconAngleRate(3);

boost::mutex IMU_Vicon::viconPosition_lock;
boost::mutex IMU_Vicon::viconVelocity_lock;
boost::mutex IMU_Vicon::viconAngle_lock;
boost::mutex IMU_Vicon::viconAngleRate_lock;

void IMU_Vicon::mavlink_handler(const lcm_recv_buf_t *rbuf, const char * channel,
		const mavconn_mavlink_msg_container_t* container, void * user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);
	
	static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint32_t messageLength = mavlink_msg_to_send_buffer(buf, msg);
	
	if(msg->msgid ==MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE)
	{
		std::cout<<"Vicon Position Message Received @IMU_Vicon"<<std::endl;
		mavlink_vicon_position_estimate_t vicon_estmate;
                mavlink_msg_vicon_position_estimate_decode(msg, &vicon_estmate);
		
		{
			boost::mutex::scoped_lock lock(viconPosition_lock);
			viconPosition(0) = (double)vicon_estmate.x;
			viconPosition(1) = (double)vicon_estmate.y;
			viconPosition(2) = (double)vicon_estmate.z;
		}
		{
			boost::mutex::scoped_lock lock(viconAngle_lock);
			viconAngle(0) = (double)vicon_estmate.roll;
			viconAngle(1) = (double)vicon_estmate.yaw;
			viconAngle(2) = (double)vicon_estmate.pitch;
		}
	}
	else if(msg->msgid ==MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE)
	{
		std::cout<<"Vicon Velocity Message Received @IMU_Vicon"<<std::endl;
		mavlink_vicon_velocity_estimate_t vicon_velocity_esmate;
		mavlink_msg_vicon_velocity_estimate_decode(msg, &vicon_velocity_esmate);
		{
			boost::mutex::scoped_lock lock(viconVelocity_lock);
			viconVelocity(0) = (double)vicon_velocity_esmate.vx;
			viconVelocity(1) = (double)vicon_velocity_esmate.vy;
			viconVelocity(2) = (double)vicon_velocity_esmate.vz;
		}
		{
			boost::mutex::scoped_lock lock(viconAngleRate_lock);
			viconAngleRate(0) = (double)vicon_velocity_esmate.vroll;
			viconAngleRate(1) = (double)vicon_velocity_esmate.vyaw;
			viconAngleRate(2) = (double)vicon_velocity_esmate.vpitch;
		}
	}	
}


blas::vector<double> IMU_Vicon::getPosition()
{
	boost::mutex::scoped_lock lock(viconPosition_lock);
	return viconPosition;
}

blas::vector<double> IMU_Vicon::getVelocity()
{
	boost::mutex::scoped_lock lock(viconVelocity_lock);
	return viconVelocity;
}

blas::vector<double> IMU_Vicon::getAngle()
{
	boost::mutex::scoped_lock lock(viconAngle_lock);
	return viconAngle;
}

blas::vector<double> IMU_Vicon::getAngleRate()
{
	boost::mutex::scoped_lock lock(viconAngleRate_lock);
	return viconAngleRate;
}

//	class IMU_read

IMU_Vicon::IMU_read::IMU_read(IMU_Vicon *parent)
{
	if(parent == NULL)
	{
		throw(bad_IMU_Vicon_parent());
	}
	this->parent = parent;
	//TODO, Connection to the main thread is needed
	MainApp::terminate.connect(IMU_Vicon::IMU_read::do_terminate());
}


boost::recursive_mutex IMU_Vicon::IMU_read::terminate_mutex;
bool IMU_Vicon::IMU_read::terminate;

bool IMU_Vicon::IMU_read::check_terminate()
{
	bool t = false;
	terminate_mutex.lock();
	t = terminate;
	terminate_mutex.unlock();
	return !t;
}

void IMU_Vicon::IMU_read::do_terminate::operator()()
{
  	terminate_mutex.lock();
  	terminate = true;
  	message() << "IMU_read: Received terminate signal. Stop the thread";
  	terminate_mutex.unlock();
}

void IMU_Vicon::IMU_read::operator()()
{
	while(check_terminate())
	{
		lcm_handle(lcm); // This function has the blocking property
//		std::cout<<"I am in a IMU_read thread"<<std::endl;
	}
}



blas::matrix<double> IMU_Vicon::angle2rotationMatrix(const blas::vector<double>& euler)
{
	blas::matrix<double> rot(3,3);
	rot.clear();

	double roll = euler[0], pitch = euler[1], yaw = euler[2];
	rot(0, 0) = cos(yaw)*cos(pitch);
	rot(0, 1) = sin(yaw)*cos(pitch);
	rot(0, 2) = -sin(pitch);
	rot(1, 0) = -sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll);
	rot(1, 1) = cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll);
	rot(1, 2) = cos(pitch)*sin(roll);
	rot(2, 0) = sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll);
	rot(2, 1) = -cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll);
	rot(2, 2) = cos(pitch)*cos(roll);

	return trans(rot);
}

