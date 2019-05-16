/*
  Created by ict on 19-3-19.
 */
#ifndef MY_SOCKET_CLIENT_ZFPCOMPRESSION_H
#define MY_SOCKET_CLIENT_ZFPCOMPRESSION_H

#include <time.h>
#include <string>
#include <string> //PCL necessary
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zfp.h> //compress x,y,z data



typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

/*
 get depth data and rgb data from file
 input : filename
 output : pointer to depthdata
 output : pointer tp rgbdata
 output : size of point
 */

 void get_raw_data_from_file(std::string Filename, float *&xyz_array, unsigned char *&rgb_array, unsigned long& size){
    //new PointCloud object for load point cloud
    PointCloud::Ptr cloud(new PointCloud);

    //load pcdfile. if failed, give error
    if (pcl::io::loadPCDFile<PointT>(Filename, *cloud) == -1) {
        PCL_ERROR("can not open file\n");
        exit(0);
    }

    //show original point cloud
    //pcl::visualization::CloudViewer viewer("original point cloud");
    //viewer.showCloud(cloud);
    //while (!viewer.wasStopped()) {
        //do something
    //}
    size = cloud->points.size();

    xyz_array = (float* ) malloc(size * sizeof(float) * 3); // for x, y, z


    rgb_array = (unsigned char*)malloc((size) * sizeof(unsigned char) * 3); // for r,g,b

    int i = 0, j = 0;
    for(auto point : cloud->points){
        xyz_array[i + 0] = (float)point.x;
        xyz_array[i + 1] = (float)point.y;
        xyz_array[i + 2] = (float)point.z;

        rgb_array[j + 0] = (unsigned char)point.r;
        rgb_array[j + 1] = (unsigned char)point.g;
        rgb_array[j + 2] = (unsigned char)point.b;

        i += 3;
        j += 3;
    }

}

/*
 get depth data and rgb data from point cloud
 input : filename
 output : pointer to depthdata
 output : pointer tp rgbdata
 output : size of point
 */
void get_raw_data_from_point_cloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, float *&xyz_array, unsigned char *&rgb_array, unsigned long& size){
    size = cloud->points.size();

    xyz_array = (float* ) malloc(size * sizeof(float) * 3); // for x, y, z


    rgb_array = (unsigned char*)malloc((size) * sizeof(unsigned char) * 3); // for r,g,b

    int i = 0, j = 0;
    for(auto point : cloud->points){
        xyz_array[i + 0] = (float)point.x;
        xyz_array[i + 1] = (float)point.y;
        xyz_array[i + 2] = (float)point.z;

        rgb_array[j + 0] = (unsigned char)point.r;
        rgb_array[j + 1] = (unsigned char)point.g;
        rgb_array[j + 2] = (unsigned char)point.b;

        i += 3;
        j += 3;
    }

}
/*
 compress [x, y, z] using zfp compression algorithm
 input : buffer
 input : size of point
 input : pointer to [x,y,z] data
 input : tolerance
 */

void my_zfp_compression_xyz(void *&buffer, unsigned int org_size, float *xyz, size_t& bytes_size, double tolerance){
    zfp_type type;
    zfp_field* field;
    zfp_stream* zfp;
    size_t  bufsize;
    bitstream* stream;
    size_t zfpsize;

    type = zfp_type_float;
    field = zfp_field_2d(xyz, type, 3, org_size);

    zfp = zfp_stream_open(NULL);

    zfp_stream_set_accuracy(zfp, tolerance);

    bufsize = zfp_stream_maximum_size(zfp, field);
    buffer = malloc(bufsize);

    stream = stream_open(buffer, bufsize);
    zfp_stream_set_bit_stream(zfp, stream);
    zfp_stream_rewind(zfp);

    //write file header
    size_t tem_size = zfp_write_header(zfp, field, ZFP_HEADER_FULL);
    if(tem_size == 0){
        cerr << "write header failed\n";
        exit(0);
    }

    //compress
    zfpsize = zfp_compress(zfp, field);
    bytes_size = zfpsize;
    if(!zfpsize){
        cerr << "compression failed!\n";
        //exit(0);
    }
}

void my_zfp_decompression_xyz(void* buffer, size_t bufsize, float *&xyz){
    bitstream *stream;
    stream = stream_open(buffer, bufsize);

    zfp_stream *zfp = zfp_stream_open(NULL);
    zfp_stream_set_bit_stream(zfp, stream);
    zfp_field *field = zfp_field_alloc();
    zfp_stream_rewind(zfp);

    size_t tem_size = zfp_read_header(zfp, field, ZFP_HEADER_FULL);
    if(tem_size == 0){
        //cerr << "read header failed\n";
        //exit(0);
    }


    float *buf = (float *)malloc(sizeof(float) * field->nx * field->ny);
    zfp_field_set_pointer(field, buf);
    zfp_decompress(zfp, field);
    xyz = (float *)field->data;

}

#endif //MY_SOCKET_CLIENT_ZFPCOMPRESSION_H
