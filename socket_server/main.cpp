/*
 *Created on 2019-3-6
 *Author : Qiu Rui
 *Version 1.0
 *Title : send compressed point cloud by socket,this is client code
 *Copyright : Copyright(c) 2019
 */

#include <iostream>
#include <stdio.h>
#include <dirent.h>
#include <vector>
#include <algorithm>
#include <time.h> //test run time
#include <sys/socket.h> //socket necessary
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <string>

//#include "pclcomression.h"
//#include "utils.h"
#include "MySimpleOpenNIViewer.h"

#include <boost/asio.hpp>

using boost::asio::ip::tcp;
using namespace std;

//main
int main(int argc, char *argv[]) {

    //static point cloud test
//    string Filename = "loot_vox10_1000.ply";
//    std::cout << Filename << std::endl;
//
//    test_zfp_jpeg(Filename);

    //test_pcl_comression(Filename);


    //real-time point cloud test
    try {
        boost::asio::io_service io_service;
        tcp::endpoint endpoint(tcp::v4(), 6666);
        tcp::acceptor acceptor(io_service, endpoint);

        tcp::iostream socketStream;

        cout << "wating for connection.." << endl;

        acceptor.accept(*socketStream.rdbuf());

        cout << "Connected!" << std::endl;

        MySimpleOpenNIViewer v (socketStream);
        v.run();

        cout << "Disconnected!" << std::endl;

        boost::this_thread::sleep(boost::posix_time::seconds(3));
    }
    catch (std::exception& e)
    {
        std::cerr << e.what () << std::endl;
    }

    return 0;
}