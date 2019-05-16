//
// Created by ict on 19-4-15.
//

#ifndef MY_SOCKET_CLIENT__JPEGCOMPRESSION_H_
#define MY_SOCKET_CLIENT__JPEGCOMPRESSION_H_

#endif //MY_SOCKET_CLIENT__JPEGCOMPRESSION_H_

#include <turbojpeg.h>
#include <setjmp.h>
#include <stdio.h>
#include <malloc.h>
#include <cmath>
#include <time.h>

using std::cout;
using std::endl;

void my_jpeg_compression_rgb(size_t PointSize, void **buffer, void *image, long unsigned int *_jpegSize){

    int ImgWidth;
    int ImgHeight;

    ImgHeight = ImgWidth = ceil(sqrt((double) PointSize));

    const int JPEG_QUALITY = 75;
    const int COLOR_COMPONENTS = 3;

    tjhandle _jpegCompressor = tjInitCompress();

    tjCompress2(_jpegCompressor, (unsigned char *)image, ImgWidth, 0, ImgHeight, TJPF_RGB,
                (unsigned char **)buffer, _jpegSize, TJSAMP_444, JPEG_QUALITY,
                TJFLAG_FASTDCT);

    tjDestroy(_jpegCompressor);

    //tjFree(&_compressedImg);
}

void my_jpeg_decompression_rgb(long unsigned int _jpegSize, void *buffer, void *&image) {
    int jpegSubsamp, width, height;

    tjhandle _jpegDecompressor = tjInitDecompress();

    tjDecompressHeader2(_jpegDecompressor, (unsigned char *)buffer, _jpegSize, &width, &height, &jpegSubsamp);

    //cout << "Debug : " << width << " " << height << endl;
    image = (unsigned char*)malloc(sizeof(unsigned char) * 3 * width * height);

    tjDecompress2(_jpegDecompressor, (unsigned char *)buffer, _jpegSize, (unsigned char *)image, width, 0/*pitch*/, height, TJPF_RGB, TJFLAG_FASTDCT);

    tjDestroy(_jpegDecompressor);
}

