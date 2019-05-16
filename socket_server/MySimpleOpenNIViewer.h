//
// Created by ict on 19-5-7.
//

#ifndef MY_SOCKET_CLIENT_CMAKE_BUILD_DEBUG_SIMPLEOPENNIVIEWER_H_
#define MY_SOCKET_CLIENT_CMAKE_BUILD_DEBUG_SIMPLEOPENNIVIEWER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>

#include <pcl/compression/octree_pointcloud_compression.h>

#include <iostream>
#include <vector>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include "stdio.h"

#include <iostream>
#include <string>

//#include "utils.h"
#include "zfpcompression.h"
#include "jpegcompression.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;

class MySimpleOpenNIViewer
{
 public:
  MySimpleOpenNIViewer (ostream& outputFile_arg) :
      //viewer ("Input Point Cloud - PCL Compression Viewer"),
      outputFile_(outputFile_arg)
  {
  }

  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
  {
      //if (!viewer.wasStopped ())
      //{
          //viewer.showCloud (cloud);
          // using zfp + jpeg compress cloud
          // convert char type to ostream
          float* xyz_array = nullptr;
          unsigned char* rgb_array = nullptr;

          size_t pointSize;
          get_raw_data_from_point_cloud(cloud, xyz_array, rgb_array, pointSize);

          //convert pointsize to string. test ok!
          unsigned long mask = 0xFF;
          char tem_point[3];
          string tem_pointsize;
          tem_point[0] = (pointSize >> 16) & mask;
          tem_point[1] = (pointSize >> 8) & mask;
          tem_point[2] = (pointSize) & mask;
          tem_pointsize.assign(tem_point, sizeof(tem_point));

          //compress xyz
          void *xyzbuffer = NULL;
          size_t bufSize;
          //clock_t start = clock();
          my_zfp_compression_xyz(xyzbuffer, (unsigned int)pointSize, xyz_array, bufSize, 10);
          //cout << "xyz compression time :" << (double)(clock() - start) / CLOCKS_PER_SEC << "s" << endl;

          //compress rgb
          void *rgbbuffer;
          long unsigned int _jpegSize = 0;
          //start1 = clock();
          my_jpeg_compression_rgb(pointSize, &rgbbuffer, rgb_array, &_jpegSize);
          //cout << "rgb compression time :" << (double)(clock() - start1) / CLOCKS_PER_SEC << "s" << endl;

          string full_stream;
          string tem_xyzbuffer;
          string tem_rgbbuffer;
          string tem_xyzsize;
          string tem_rgbsize;

          //covert bufsize to binary format and save with char type
          char tem_xyz[3];
          tem_xyz[0] = (bufSize >> 16) & mask;
          tem_xyz[1] = (bufSize >> 8) & mask;
          tem_xyz[2] = (bufSize) & mask;
          tem_xyzsize.assign(tem_xyz, 3);
          tem_xyzbuffer.assign((char *)xyzbuffer, bufSize);



          char tem_rgb[3];
          tem_rgb[0] = (_jpegSize >> 16) & mask;
          tem_rgb[1] = (_jpegSize >> 8) & mask;
          tem_rgb[2] = (_jpegSize) & mask;
          tem_rgbsize.assign(tem_rgb, 3);
          tem_rgbbuffer.assign((char *)rgbbuffer, _jpegSize);


          //convert string to ostream for transmission
          full_stream = tem_pointsize;

          full_stream.append(tem_xyzsize).append(tem_xyzbuffer).append(tem_rgbsize).append(tem_rgbbuffer);

          cout << "full stream size : " << full_stream.size() << endl;

          outputFile_ << full_stream;


      //}
  }

  void run ()
  {

      // create a new grabber for OpenNI devices
      pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

      // make callback function from member function
      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
          boost::bind (&MySimpleOpenNIViewer::cloud_cb_, this, _1);

      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c = interface->registerCallback (f);

      // start receiving point clouds
      interface->start ();


      while (!outputFile_.fail())
      {
          boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      interface->stop ();
  }

  //pcl::visualization::CloudViewer viewer;
  ostream& outputFile_;
};
#endif //MY_SOCKET_CLIENT_CMAKE_BUILD_DEBUG_SIMPLEOPENNIVIEWER_H_
