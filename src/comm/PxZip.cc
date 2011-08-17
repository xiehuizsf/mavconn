#include "PxZip.h"

#include <cassert>
#include <zlib.h>

void
PxZip::compress(uint8_t* inData, size_t inDataSize,
				std::vector<uint8_t>& outData)
{
	std::vector<uint8_t> buffer;

	const size_t kBufferSize = 128 * 1024;
	uint8_t tempBuffer[kBufferSize];

	z_stream strm;
	strm.zalloc = 0;
	strm.zfree = 0;
	strm.next_in = inData;
	strm.avail_in = inDataSize;
	strm.next_out = tempBuffer;
	strm.avail_out = kBufferSize;
	
	deflateInit(&strm, Z_BEST_SPEED);

	while (strm.avail_in != 0)
	{
 		int res = deflate(&strm, Z_NO_FLUSH);
 		assert(res == Z_OK);
 		if (strm.avail_out == 0)
 		{
 			buffer.insert(buffer.end(), tempBuffer, tempBuffer + kBufferSize);
   			strm.next_out = tempBuffer;
			strm.avail_out = kBufferSize;
		}
	}

	int deflateRes = Z_OK;
	while (deflateRes == Z_OK)
	{
		if (strm.avail_out == 0)
		{
			buffer.insert(buffer.end(), tempBuffer, tempBuffer + kBufferSize);
			strm.next_out = tempBuffer;
			strm.avail_out = kBufferSize;
		}
		deflateRes = deflate(&strm, Z_FINISH);
	}

	assert(deflate_res == Z_STREAM_END);
	buffer.insert(buffer.end(), tempBuffer, tempBuffer + kBufferSize - strm.avail_out);
	deflateEnd(&strm);

	outData.swap(buffer);
}			

void
PxZip::decompress(uint8_t* inData, size_t inDataSize,
				  std::vector<uint8_t>& outData)
{
	std::vector<uint8_t> buffer;

	const size_t kBufferSize = 128 * 1024;
	uint8_t tempBuffer[kBufferSize];

	z_stream strm;
	strm.zalloc = 0;
	strm.zfree = 0;
	strm.next_in = inData;
	strm.avail_in = inDataSize;
	strm.next_out = tempBuffer;
	strm.avail_out = kBufferSize;
	
	inflateInit(&strm);

	while (strm.avail_in != 0)
	{
 		int res = inflate(&strm, Z_NO_FLUSH);
 		assert(res == Z_OK);
 		if (strm.avail_out == 0)
 		{
 			buffer.insert(buffer.end(), tempBuffer, tempBuffer + kBufferSize);
   			strm.next_out = tempBuffer;
			strm.avail_out = kBufferSize;
		}
	}

	int inflateRes = Z_OK;
	while (inflateRes == Z_OK)
	{
		if (strm.avail_out == 0)
		{
			buffer.insert(buffer.end(), tempBuffer, tempBuffer + kBufferSize);
			strm.next_out = tempBuffer;
			strm.avail_out = kBufferSize;
		}
		inflateRes = inflate(&strm, Z_FINISH);
	}

	assert(inflate_res == Z_STREAM_END);
	buffer.insert(buffer.end(), tempBuffer, tempBuffer + kBufferSize - strm.avail_out);
	inflateEnd(&strm);

	outData.swap(buffer);			  
}
