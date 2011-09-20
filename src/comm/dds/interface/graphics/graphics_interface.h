#ifndef GRAPHICS_INTERFACE_H
#define GRAPHICS_INTERFACE_H

#include "../../Topic.h"
#include "dds_gl_overlay_message_tPlugin.h"
#include "dds_gl_overlay_message_tSupport.h"

namespace px
{

/**
 * Defines the type specific interface for dds_gl_overlay_message_t.
 */
class GLOverlayTopic: public Topic< dds_gl_overlay_message_t,
									dds_gl_overlay_message_tTypeSupport,
									dds_gl_overlay_message_tDataReader,
									dds_gl_overlay_message_tDataWriter >
{
public:
	/**
	* Returns a handle to the interface instance.
	* @return The instance handle
	*/
	static GLOverlayTopic* instance(void);

protected:
	/**
	* A constructor. Does nothing.
	*/
	GLOverlayTopic();

private:
	/**
	* The handle to the singleton instance.
	*/
	static GLOverlayTopic* _instance;
};

}

#endif
