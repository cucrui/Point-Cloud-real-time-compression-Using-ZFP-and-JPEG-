//
// Created by ict on 19-3-19.
//

#ifndef MY_SOCKET_CLIENT_PCLCOMRESSION_H
#define MY_SOCKET_CLIENT_PCLCOMRESSION_H

#include <time.h>
#include <string> ///PCL necessary
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include "quality_metrics.h"
/*
 * compression with pcl
 * input : filename
 * output : comression bytes size and pointer to compression
 */

void test_pcl_comression(string Filename){

    //new PointCloud object for load point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //load pcdfile. if failed, give error
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(Filename, *cloud) == -1) {
        PCL_ERROR("can not open file\n");
        exit(0);
    }

    //set octree resolution
    float resolution = 128.0f;
    pcl::octree::OctreePointCloud<pcl::PointXYZRGB> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    //define a encoder function pointer
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> *PointCloudEncoder;

    //set compression configuration
    pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR;

    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfile, true);



    //define output string and compressed bytes stream
    std::stringstream compressionData;

    //start compression

    clock_t encode_time_st = clock();
    PointCloudEncoder->encodePointCloud(cloud, compressionData);
    std::cout << "Test PCL compression Encode time : " << (double)(clock() - encode_time_st) / CLOCKS_PER_SEC << "s" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> *PointCloudDecoder;
    PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> ();

    PointCloudDecoder->decodePointCloud(compressionData, cloudout);


    cout << "pcl compression metric : " << endl;
    QualityMetric metric;
    computeQualityMetric<pcl::PointXYZRGB>(*cloud, *cloudout, metric);

}



#endif //MY_SOCKET_CLIENT_PCLCOMRESSION_H
