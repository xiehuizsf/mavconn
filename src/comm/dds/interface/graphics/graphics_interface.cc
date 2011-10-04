#include "graphics_interface.h"

namespace px
{

const char* GL_OVERLAY_NAME = "gl_overlay";

GLOverlayTopic* GLOverlayTopic::_instance = 0;

GLOverlayTopic* GLOverlayTopic::instance()
{
	if (_instance == 0)
	{
		_instance = new GLOverlayTopic;
		_instance->topicName.assign(GL_OVERLAY_NAME);
		_instance->topicType = TOPIC_PUBLISH_SUBSCRIBE;
		_instance->plugin = dds_gl_overlay_message_tPlugin_new();
	}
	return _instance;
}

GLOverlayTopic::GLOverlayTopic() { };

}
