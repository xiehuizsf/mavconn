#include <iostream>
#include <mavconn.h>
#include <ndds/ndds_cpp.h>

#include "dds/Middleware.h"
#include "dds/interface/mavlink/mavlink_interface.h"

bool quit = false;

void signalHandler(int signal)
{
	if (signal == SIGINT)
	{
		fprintf(stderr, "# INFO: Shutting down...\n");
		quit = true;
	}
}

void
mavlinkLCMHandler(const lcm_recv_buf_t* rbuf, const char* channel,
				  const mavlink_message_t* msg, void* user)
{
	// forward MAVLINK messages from LCM to DDS
	dds_mavlink_message_t* dds_msg;
	dds_mavlink_message_t_initialize(dds_msg);

	dds_msg->len = msg->len;
	dds_msg->seq = msg->seq;
	dds_msg->sysid = msg->sysid;
	dds_msg->compid = msg->compid;
	dds_msg->msgid = msg->msgid;
	memcpy(dds_msg->payload, msg->payload, 255);
	dds_msg->ck_a = msg->ck_a;
	dds_msg->ck_b = msg->ck_b;

	px::MavlinkTopic::instance()->publish(dds_msg);

	dds_mavlink_message_t_finalize(dds_msg);
}

void
mavlinkDDSHandler(void* msg, lcm_t* lcm)
{
	dds_mavlink_message_t* dds_msg = reinterpret_cast<dds_mavlink_message_t*>(msg);

	// forward MAVLINK messages from DDS to LCM
	mavlink_message_t lcm_msg;

	lcm_msg.len = dds_msg->len;
	lcm_msg.seq = dds_msg->seq;
	lcm_msg.sysid = dds_msg->sysid;
	lcm_msg.compid = dds_msg->compid;
	lcm_msg.msgid = dds_msg->msgid;
	memcpy(lcm_msg.payload, dds_msg->payload, 255);
	lcm_msg.ck_a = dds_msg->ck_a;
	lcm_msg.ck_b = dds_msg->ck_b;

	mavlink_message_t_publish(lcm, "MAVLINK", &lcm_msg);
}

int
main(int argc, char** argv)
{
	enum
	{
		LCM_TO_DDS,
		DDS_TO_LCM
	};

	// Find option for bridge mode
	Glib::OptionGroup optGroup("options", "options", "Configuration options");

	Glib::OptionEntry optBridgeMode;
	optBridgeMode.set_short_name('m');
	optBridgeMode.set_long_name("mode");
	optBridgeMode.set_description("dds2lcm: Push DDS messages to LCM, lcm2dds: Push LCM messages to DDS");

	std::string bridgeMode;
	optGroup.add_entry_filename(optBridgeMode, bridgeMode);

	Glib::OptionContext optContext("");
	optContext.set_help_enabled(true);
	optContext.set_ignore_unknown_options(true);
	optContext.set_main_group(optGroup);

	try
	{
		if (!optContext.parse(argc, argv))
		{
			fprintf(stderr, "# ERROR: Cannot parse options.\n");
			return 1;
		}
	}
	catch (Glib::OptionError& error)
	{
		fprintf(stderr, "# ERROR: Cannot parse options.\n");
		return 1;
	}

	bool dds2lcm = false;
	bool lcm2dds = false;
	if (bridgeMode.compare("dds2lcm") == 0)
	{
		dds2lcm = true;
	}
	if (bridgeMode.compare("lcm2dds") == 0)
	{
		lcm2dds = true;
	}
	if (!dds2lcm && !lcm2dds)
	{
		fprintf(stderr, "# ERROR: Bridge mode is not valid. Please use either dds2lcm or lcm2dds\n");
		return 1;
	}

	signal(SIGINT, signalHandler);

	lcm_t* lcm = lcm_create("udpm://");
	if (!lcm)
	{
		fprintf(stderr, "# ERROR: Cannot create LCM instance.\n");
		exit(EXIT_FAILURE);
	}

	px::Middleware mw;
	mw.init(argc, argv);

	mavlink_message_t_subscription_t* mavlinkLCMSub = 0;
	if (lcm2dds)
	{
		mavlinkLCMSub = mavlink_message_t_subscribe(lcm, "MAVLINK", &mavlinkLCMHandler, 0);
		px::MavlinkTopic::instance()->advertise();
	}

	if (dds2lcm)
	{
		px::Handler handler(sigc::bind(sigc::ptr_fun(mavlinkDDSHandler), lcm));
		px::MavlinkTopic::instance()->subscribe(handler, px::SUBSCRIBE_ALL);
	}

	// listen for LCM messages
	int lcm_fd = lcm_get_fileno(lcm);

	// wait a limited amount of time for an incoming LCM message
	struct timeval timeout = {
		1,	// seconds
		0	// microseconds
	};

	while (!quit)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(lcm_fd, &fds);

		int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);

		if (status != 0 && FD_ISSET(lcm_fd, &fds) && !quit)
		{
			// LCM has events ready to be processed.
			lcm_handle(lcm);
		}
	}

	mw.shutdown();

	if (lcm2dds)
	{
		mavlink_message_t_unsubscribe(lcm, mavlinkLCMSub);
	}

	return 0;
}
