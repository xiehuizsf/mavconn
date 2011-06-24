#ifndef PXSHM_H
#define PXSHM_H

#include <sys/ipc.h>
#include <sys/shm.h>
#include <string>
#include <vector>

class PxSHM
{
public:
	typedef enum
	{
		CAMERA_FORWARD_LEFT = 0x01,
		CAMERA_FORWARD_RIGHT = 0x02,
		CAMERA_DOWNWARD_LEFT = 0x04,
		CAMERA_DOWNWARD_RIGHT = 0x08,
		CAMERA_NONE = 0x10
	} CameraPosition;

	typedef enum
	{
		CAMERA_MONO_8 = 0,
		CAMERA_MONO_24 = 1,
		CAMERA_STEREO_8 = 2,
		CAMERA_STEREO_24 = 3,
		CAMERA_KINECT = 4
	} CameraType;

	typedef enum
	{
		SERVER_TYPE = 0,
		CLIENT_TYPE = 1
	} Type;

	PxSHM();
	~PxSHM();

	bool init(int key, Type type, int infoPacketSize, int infoQueueLength,
			  int dataPacketSize, int dataQueueLength);

	int hashKey(const std::string& str) const;

	int readInfoPacket(std::vector<uint8_t>& data);

	int writeInfoPacket(const std::vector<uint8_t>& data);

	int readDataPacket(std::vector<uint8_t>& data);

	int writeDataPacket(const std::vector<uint8_t>& data);

	bool bytesWaiting(void) const;

	long long getMax(void) const;

	bool setMax(long long max) const;

	Type getType(void) const;

private:
	typedef enum {
		READ_INFO = 0,
		WRITE_INFO = 1,
		READ_DATA = 2,
		WRITE_DATA = 3
	} Mode;

	uint8_t crc(const std::vector<uint8_t>& data) const;

	int pos(int num, Mode mode) const;

	void copyToSHM(const uint8_t* data, int len, int off);
	void copyToSHM(const std::vector<uint8_t>& data, int off);
	void copyFromSHM(uint8_t* data, int len, int off) const;
	void copyFromSHM(std::vector<uint8_t>& data, int len, int off) const;

	Type              type;      /* server/client type */
	unsigned char   * mem;       /* shared memory segment */
	unsigned int      key;       /* shared memory key */
	unsigned int      i_size;    /* size of the (static) info buffer */
	unsigned int      d_size;    /* size of the (ringbuffer) data buffer */
	unsigned int      w_off;     /* write offset */
	unsigned int      r_off;     /* read offset */
	unsigned int      i_off;     /* info offset */
};

#endif
