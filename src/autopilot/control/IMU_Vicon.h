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
#ifndef IMU_VICON_H
#define IMU_VICON_H

/* Boost Headers */
#include <boost/thread.hpp>
#include <boost/signals2.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "mavconn.h"
#include "autopilot.h"
#include "MainApp.h"


/**
   \brief This class implements the retrieving the virtual IMU data from the process of mavconn-bridge-udp.

   \author Hui Xie <xie1@ualberta.ca>
   \date May 21, 2013: Creation of class
 */

class IMU_Vicon
{
public:
	//Return the instance of the IMUVicon object using the singleton design pattern
	static IMU_Vicon* getInstance();
	blas::vector<double> getPosition();
	blas::vector<double> getVelocity();
	blas::vector<double> getAngle();
	blas::vector<double> getAngleRate();
	static blas::matrix<double> angle2rotationMatrix(const blas::vector<double>& euler);

private:
	// Constructor
	IMU_Vicon();
	// Desctructor: Free Memory 
	~IMU_Vicon();
	static IMU_Vicon* _instance;
	static boost::mutex _instance_lock;
	/// Exception to be thrown if LogFileWrite is constructed without a valid pointer to a parent
	class bad_IMU_Vicon_parent : public std::exception
	{
	  virtual const char* what() const throw()
		{
		  return "IMU_Vicon_read requires non-null LogFile parent";
		}
	};


	class IMU_read
	{
	public:
		IMU_read(IMU_Vicon *parent = NULL);
		void operator()();
	private:
		IMU_Vicon* parent;
		static boost::recursive_mutex terminate_mutex;
		static bool terminate;
		bool check_terminate();
		class do_terminate
		{
		public:
		  /// set LogFile::LogFileWrite::terminate to true
		  void operator()();
		};
	};

	/// store the current position

	static blas::vector<double> viconPosition;	
	/// serialize access to Vicon::position
	static boost::mutex viconPosition_lock;

	/// store current velocity in ned coords
	static blas::vector<double>  viconVelocity;
	/// serialize access to IMU::velocity
	static boost::mutex viconVelocity_lock;


	static blas::vector<double> viconAngle;	
	/// serialize access to Vicon::position
	static boost::mutex viconAngle_lock;

	/// store current velocity in ned coords
	static blas::vector<double> viconAngleRate;
	/// serialize access to IMU::velocity
	static boost::mutex viconAngleRate_lock;


	static void mavlink_handler(const lcm_recv_buf_t *rbuf, const char * channel,
		const mavconn_mavlink_msg_container_t* container, void * user);

	
	boost::thread IMU_thread;

	static mavconn_mavlink_msg_container_t_subscription_t* comm_sub;
	static lcm_t* lcm;
};
#endif
