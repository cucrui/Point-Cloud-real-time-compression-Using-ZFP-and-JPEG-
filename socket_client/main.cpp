

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

#include <boost/asio.hpp>

#include "jpegcompression.h"
#include "zfpcompression.h"

using boost::asio::ip::tcp;

//using namespace pcl;
//using namespace pcl::io;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

//main
int main(){
    string hostname = "localhost";

    cout << "Connecting to:" << hostname << ".." << endl;

    try {
        tcp::iostream socketStream(hostname.c_str(), "6666");

        std::cout << "Connected!" << endl;

        pcl::visualization::CloudViewer viewer("Decoded Point cloud - ZFP + JPEG compression");

        while (!socketStream.fail()) {
            FPS_CALC("drawing");
            PointCloud::Ptr cloudOut(new PointCloud);

            //get point size
            size_t pointsize = 0x00;
            char tem_point[3];
            socketStream.read(tem_point, 3);

            pointsize = ((pointsize|(uint8_t)tem_point[0])<<8);
            pointsize = ((pointsize|(uint8_t)tem_point[1])<< 8);
            pointsize = pointsize  |(uint8_t)tem_point[2];

            //cout << "debug : point size = " << pointsize << endl;

            //get xyz buffer size
            size_t bufsize_xyz = 0x00;
            char tem_xyz[3];

            socketStream.read(tem_xyz, 3);

            bufsize_xyz = ((bufsize_xyz|(uint8_t)tem_xyz[0])<<8);
            bufsize_xyz = ((bufsize_xyz|(uint8_t)tem_xyz[1])<< 8);
            bufsize_xyz = bufsize_xyz  |(uint8_t)tem_xyz[2];

            //cout << "debug : bufsize_xyz = " << bufsize_xyz << endl;

            //get xyz buffer
            char* xyz_buf = (char*)malloc(bufsize_xyz);
            socketStream.read(xyz_buf, bufsize_xyz);

            //get rgb buffer size
            size_t bufsize_rgb = 0x00;
            char tem_rgb[3];
            socketStream.read(tem_rgb, 3);
            bufsize_rgb = ((bufsize_rgb|(uint8_t)tem_rgb[0])<<8);
            bufsize_rgb = ((bufsize_rgb|(uint8_t)tem_rgb[1])<< 8);
            bufsize_rgb = bufsize_rgb  |(uint8_t)tem_rgb[2];

            //cout << "debug : bufsize_rgb = " << bufsize_rgb << endl;

            //get rgb buffer
            char* rgb_buf = (char*)malloc(bufsize_rgb);
            socketStream.read(rgb_buf, bufsize_rgb);

            //decode (x,y,z) and (rgb)
            float *dec_xyz = nullptr;
            void *dec_rgb = nullptr;

            my_zfp_decompression_xyz((void *)xyz_buf, bufsize_xyz, dec_xyz);
            my_jpeg_decompression_rgb(bufsize_rgb, (void *)rgb_buf, dec_rgb);

            //get full point cloud
            for(int i = 0, j = 0; j < pointsize; i += 3, j++){
                PointT p;
                p.x = dec_xyz[i];
                p.y = dec_xyz[i+1];
                p.z = dec_xyz[i+2];

                p.r = ((unsigned char *)dec_rgb)[i];
                p.g = ((unsigned char *)dec_rgb)[i+1];
                p.b = ((unsigned char *)dec_rgb)[i+2];
                cloudOut->points.push_back(p);
            }
            cloudOut->is_dense = false;

            viewer.showCloud(cloudOut);
            cloudOut->clear();
            //system("pause");
        }
    }
    catch (std::exception& e){
        std::cout << "Exception: " << e.what () << std::endl;
    }
    return 0;
}

///*
// *Created on 2019-3-6
// *Author : Qiu Rui
// *Version 1.0
// *Title : recieve compressed point cloud by socket and
// * view it,this is server code
// *Copyright : Copyright(c) 2019
// */
//
//#include <cstdio>
//#include <string.h>
//#include <unistd.h>
//#include <time.h>
//#include <arpa/inet.h> ///socket necessary
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <printf.h>
//#include <stdlib.h>
//
//#include <string> ///PCL necessary
//#include <sstream>
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/compression/octree_pointcloud_compression.h>
//
//#define MAXPENDING 5
//#define RCVBUFSIZE 1024
//
//clock_t comm_time_st;
//clock_t comm_time_ed;
//clock_t decode_time_st;
//clock_t decode_time_ed;
//
//void DieWithError(const char *erroMessage){
//    perror(erroMessage);
//    exit(1);
//}
//
//void HandleTCPClient(int clntSock){
//    ///define rcvBuffer
//    char rcvBuffer[RCVBUFSIZE];
//    int recvMsgSize = 0;
//
//    ///clear rcvBuffer
//    bzero(rcvBuffer, RCVBUFSIZE);
//
//    ///this string for converting to stringstream
//    string byteStream;
//
//    while(recvMsgSize = recv(clntSock, rcvBuffer, RCVBUFSIZE, 0)){
//        if(recvMsgSize < 0){
//            DieWithError("Recieved data failed");
//        }
//
//        ///this ten string for adding to final byteStream
//        string tem;
//
//        ///this place is better to use string.assign()
//        ///because in rcvbuffer there are many '\0' and may lost this character by other methods
//        tem.assign(rcvBuffer, sizeof(rcvBuffer));
//
//        ///append tem string yo final byteStream
//        byteStream.append(tem);
//
//        ///clear rcvbuffer
//        bzero(rcvBuffer, RCVBUFSIZE);
//    }
//
//    comm_time_ed = clock();
//    std::cout << "communication time : " << (double)(comm_time_ed - comm_time_st) / CLOCKS_PER_SEC << "s" << std::endl;
//    std::cout << "debug : transmission finished" << std::endl;
//
//    ///define a stringstream for decoded
//    stringstream ss;
//    ss << byteStream;
//
//    ///some define for decoding
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB>());
//    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> *PointCloudDecoder;
//    PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> ();
//
//    decode_time_st = clock();
//    PointCloudDecoder->decodePointCloud(ss, cloudOut);
//    decode_time_ed = clock();
//
//    std::cout << "decode time : " << (double)(decode_time_ed - decode_time_st) / CLOCKS_PER_SEC << "s" << std::endl;
//
//    std::cout << "debug : decoded finished" << std::endl;
//
//    ///visualize decoded pointcloud
//    pcl::visualization::CloudViewer viewer("my_viewer");
//    viewer.showCloud(cloudOut);
//    while (!viewer.wasStopped()) {
//        //do something
//    }
//    close(clntSock);
//}
//
//int main(int argc, char *argv[]) {
//    int serverSock;
//    int clntSock;
//
//    struct sockaddr_in serv_addr;
//    struct sockaddr_in clnt_addr;
//
//    unsigned short servPort;
//    unsigned  int clntLen;
//
//    if(argc != 2){
//        fprintf(stderr, "Usage: %s <Server port>\n", argv[0]);
//        exit(1);
//    }
//
//    servPort = atoi(argv[1]);
//
//    /// create a socket
//    if((serverSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
//        DieWithError("socket() failed");
//
//    ///set server address
//    memset(&serv_addr, 0, sizeof(serv_addr));
//    serv_addr.sin_family = AF_INET;
//    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
//    serv_addr.sin_port = htons(servPort);
//
//    ///bind socket with server address
//    if(bind(serverSock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
//        DieWithError("bind() failed");
//
//    ///keep listening
//    if(listen(serverSock, MAXPENDING) < 0)
//        DieWithError("listen() failed");
//
//    ///waiting for connection
//    while(true){
//
//        clntLen = sizeof(clnt_addr);
//
//        ///build connection
//        comm_time_st = clock();
//        if((clntSock = accept(serverSock, (struct sockaddr * )&serv_addr, &clntLen)) < 0)
//            DieWithError("accept() failed");
//
//        printf("Handling client %s\n", inet_ntoa(clnt_addr.sin_addr));
//
//        ///handle this connection
//        HandleTCPClient(clntSock);
//    }
//
//    close(serverSock);
//    return 0;
//}



