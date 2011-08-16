#ifndef PXZIP_H
#define PXZIP_H

#include <inttypes.h>
#include <vector>

class PxZip
{
public:
	static void compress(uint8_t* inData, size_t inDataSize,
						 std::vector<uint8_t>& outData);

	static void decompress(uint8_t* inData, size_t inDataSize,
						   std::vector<uint8_t>& outData);
};

#endif
