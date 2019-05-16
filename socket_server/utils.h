//
// Created by ict on 19-3-25.
//

#ifndef MY_SOCKET_CLIENT_UTILS_H
#define MY_SOCKET_CLIENT_UTILS_H

#include <string>
#include <cstring>
#include <vector>
#include <dirent.h>
#include <algorithm>
#include <pcl/visualization/cloud_viewer.h>
#include "jpegcompression.h"
//#include "pclcomression.h"
#include "zfpcompression.h"
#include "quality_metrics.h"

using namespace std;


/*
 * test_zfp+jpeg
 */

void test_zfp_jpeg(string filename){
    float* xyz_array = nullptr;
    unsigned char* rgb_array = nullptr;

    float *decom_xyz = nullptr;
    void *decom_rgb = nullptr;

    size_t pointSize;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    get_raw_data_from_file(filename, xyz_array, rgb_array, pointSize, cloud);

    size_t bufSize;

    //compress xyz
    void *xyzbuffer = NULL;

    double tot_time;
    clock_t start = clock();
    my_zfp_compression_xyz(xyzbuffer, (unsigned int)pointSize, xyz_array, bufSize, 1);
   tot_time =  (double)(clock() - start) / CLOCKS_PER_SEC;

    my_zfp_decompression_xyz(xyzbuffer, bufSize, decom_xyz);



     //compress rgb
    void *rgbbuffer;
    long unsigned int _jpegSize = 0;
    clock_t start_1 = clock();
    my_jpeg_compression_rgb(pointSize, &rgbbuffer, rgb_array, &_jpegSize);
    tot_time += (double)(clock() - start_1) / CLOCKS_PER_SEC;

    my_jpeg_decompression_rgb(_jpegSize, rgbbuffer, decom_rgb);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i = 0, j = 0; j < pointSize; i += 3, j++){
        pcl::PointXYZRGB p;
        p.x = decom_xyz[i];
        p.y = decom_xyz[i+1];
        p.z = decom_xyz[i+2];

        p.r = ((unsigned char *)decom_rgb)[i];
        p.g = ((unsigned char *)decom_rgb)[i+1];
        p.b = ((unsigned char *)decom_rgb)[i+2];
        cloudout->points.push_back(p);
    }
    cloudout->is_dense = false;


    cout << "zfp+jpeg compression information : \n";
    //cout << "org total bytes : " << pointSize * 15 << endl;
    //cout << "compressed bytes : " << bufSize + _jpegSize << endl;
    cout << "total compression ratio : " << (pointSize * 15) / (double)(bufSize + _jpegSize) << endl;
    //cout << "org xyz bytes : " << pointSize * 12 << endl;
    //cout << "compressed xyz bytes : " << bufSize << endl;
    //cout << "xyz compression ratio : " << (pointSize * 12) / (double)(bufSize) << endl;
    //cout << "org rgb bytes : " << pointSize * 3 << endl;
    //cout << "compressed rgb bytes : " << _jpegSize << endl;
    //cout << "rgb compression ratio : " << (pointSize * 3) / (double)(_jpegSize) << endl;
    cout << "bytes per point : " << (bufSize + _jpegSize)/(double)pointSize << endl;
    cout << "total compression time : " << tot_time << " s" << endl;

    cout << "zfp compression metric : " << endl;
    QualityMetric metric;
    computeQualityMetric<pcl::PointXYZRGB>(*cloud, *cloudout, metric);

    pcl::visualization::CloudViewer viewer("my_viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
        //do something
    }
}
#endif //MY_SOCKET_CLIENT_UTILS_H
