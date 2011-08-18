#ifndef PXZIP_H
#define PXZIP_H

#include <opencv2/core/core.hpp>
#include <vector>

typedef void* tjhandle;

class PxZip
{
public:
	static PxZip* instance(void);

	PxZip();
	~PxZip();

	// use zlib to compress/decompress generic data
	void compressData(unsigned char* inData, size_t inDataSize,
					  std::vector<unsigned char>& outData);

	void decompressData(unsigned char* inData, size_t inDataSize,
						std::vector<unsigned char>& outData);

	// use libjpeg-turbo to compress/decompress image data
	void compressImage(const cv::Mat& inData,
					   std::vector<unsigned char>& outData);

	void decompressImage(unsigned char* inData, size_t inDataSize,
						 cv::Mat& outData);

private:
	static PxZip* mInstance;

	unsigned long jpegBufferSize;
	unsigned char* jpegBuffer;

	tjhandle handleCompress;
	tjhandle handleDecompress;

	const size_t kChunkSize;
	unsigned char* chunkBuffer;
};

#endif
