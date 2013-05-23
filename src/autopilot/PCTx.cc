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

#include "PCTx.h"

#define TRANSFER_TIMEOUT 200

#define DEBUG

#ifdef DEBUG
#include <iostream>
using namespace std;
#endif

PCTx* PCTx::_instance;
boost::mutex PCTx::_instance_lock;
PCTx* PCTx::getInstance()
{
	boost::mutex::scoped_lock lock(_instance_lock);
	if (!_instance)
		_instance = new PCTx;
	return _instance;
}

//PCTx::PCTx():raw_outputs(32)
PCTx::PCTx()
{
//TODO tried to adjust the initial value
	raw_outputs.resize(32);
	raw_outputs[0] = 0x00;
	raw_outputs[1] = 0x00;
	raw_outputs[2] = 0x00;
	raw_outputs[3] = 0x00;
	raw_outputs[4] = 0x00;
	raw_outputs[5] = 0x00;
	raw_outputs[6] = 0x00;
	raw_outputs[7] = 0x00;
	raw_outputs[8] = 0x00;
	raw_outputs[9] = 0x00;
	raw_outputs[10] = 0x00;
	raw_outputs[11] = 0x00;
	raw_outputs[12] = 0x00;
	raw_outputs[13] = 0x00;
	raw_outputs[14] = 0x00;
	raw_outputs[15] = 0x00;
	raw_outputs[16] = 0x00;
	raw_outputs[17] = 0x00;
	raw_outputs[18] = 0x00;
	raw_outputs[19] = 0x00;
	raw_outputs[20] = 0x00;
	raw_outputs[21] = 0x00;
	raw_outputs[22] = 0x00;
	raw_outputs[23] = 0x00;
	raw_outputs[24] = 0x00;
	raw_outputs[25] = 0x00;
	raw_outputs[26] = 0x00;
	raw_outputs[27] = 0x44;
	raw_outputs[28] = 0x00;
	raw_outputs[29] = 0x0e;
	raw_outputs[30] = 0x00;
	raw_outputs[31] = 0x28;

	usbBusy = false;
}

PCTx::~PCTx()
{

}

//TODO to change the return value;
bool PCTx::writeToDevice(void)
{
	int transferred;
	unsigned char input[10];
	std::vector<uint8_t> output(raw_outputs);
	if(!usbBusy)
	{
		{		
			boost::mutex::scoped_lock(usbBusy_lock);
			usbBusy = true;
		}
		libusb_interrupt_transfer(usb_device_handle, 0x01, &output[0], output.size(), &transferred, TRANSFER_TIMEOUT);

		if (transferred < 32)
		{
			#ifdef DEBUG
			cout << "Write failed" << endl;
			#endif
			return false;
		}

		{		
			boost::mutex::scoped_lock(usbBusy_lock);
			usbBusy = false;
		}

		//TODO to check whether the receiving code can be deleted
		libusb_interrupt_transfer(usb_device_handle, 0x81, input, 10, &transferred, TRANSFER_TIMEOUT);

		if (transferred < 10)
		{
			#ifdef DEBUG
			cout << "Read failed" << endl;
			#endif
			return false;
		}

		return true;
	}
//TODO enum some type
//	else 
//	return USB_BUSY
	
}
