/************************************************************************
* File:	RunTracker.cpp
* Brief: C++ demo for paper: Kaihua Zhang, Lei Zhang, Ming-Hsuan Yang,"Real-Time Compressive Tracking," ECCV 2012.
* Version: 1.0
* Author: Yang Xian
* Email: yang_xian521@163.com
* Date:	2012/08/03
* History:
* Revised by Kaihua Zhang on 14/8/2012, 23/8/2012
* Email: zhkhua@gmail.com
* Homepage: http://www4.comp.polyu.edu.hk/~cskhzhang/
* Project Website: http://www4.comp.polyu.edu.hk/~cslzhang/CT/CT.htm
************************************************************************/
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <memory>
#include "CompressiveTracker.h"
#include "imagemanager.h"
#include "Kalman.h"

using namespace std;

std::shared_ptr<MyKalmanFilter::KalmanFilter> kf;
int x_rect, y_rect, w_rect, h_rect;
float dt;
std::string directoryName;

void usage(const char *argv)
{
    std::cout << "Usage: " << argv << " -kalman x_rect y_rect w_rect y_rect dt_kalman directory_of_images" << std::endl;
    std::cout << "\t " << argv << " -orig x_rect y_rect w_rect y_rect directory_of_images" << std::endl;
    exit(-1);
}

void check_parameters(const int &argc, char *argv[])
{
    std::string arguments(argv[1]);

    std::cout << argc << std::endl;
    if(arguments == "-kalman" &&
            argc == 8)
    {
        std::string fs(argv[6]);
        dt = std::stof(fs);
        std::cout << "dt: " << dt << std::endl;
        directoryName = std::string(argv[7]);
    }
    else if(arguments == "-orig" &&
            argc == 7)
    {
        directoryName = std::string(argv[6]);
    }
    else
    {
        usage(argv[0]);
    }

    x_rect = atoi(argv[2]);
    y_rect = atoi(argv[3]);
    w_rect = atoi(argv[4]);
    h_rect = atoi(argv[5]);
    std::cout << "x_rect: " << x_rect << std::endl;
    std::cout << "y_rect: " << y_rect << std::endl;
    std::cout << "w_rect: " << w_rect << std::endl;
    std::cout << "h_rect: " << h_rect << std::endl;
    std::cout << "directory: " << directoryName << std::endl;
}

int main(int argc, char * argv[])
{

    check_parameters(argc, argv);

	
    cv::Rect box(x_rect, y_rect, w_rect, h_rect); // [x y width height] tracking position

    ImageManager images(directoryName);


    //Kalman Filter
    kf = std::shared_ptr<MyKalmanFilter::KalmanFilter>(new MyKalmanFilter::KalmanFilter(box.x, box.y, dt));

	// CT framework
    CompressiveTracker ct(kf);

    cv::Mat frame;
    cv::Mat grayImg;

    frame = imread(images.next(1));
    cv::cvtColor(frame, grayImg, CV_RGB2GRAY);
	ct.init(grayImg, box);    

	char strFrame[20];

    FILE* resultStream;
	resultStream = fopen("TrackingResults.txt", "w");
	fprintf (resultStream,"%i %i %i %i\n",(int)box.x,(int)box.y,(int)box.width,(int)box.height);

    for(int i = 1; i < images.getEnd()-1; i ++)
	{
        		
        frame = cv::imread(images.next(1));// get frame
        cv::cvtColor(frame, grayImg, CV_RGB2GRAY);
		
		ct.processFrame(grayImg, box);// Process frame
		
        cv::rectangle(frame, box, cv::Scalar(200,0,0),2);// Draw rectangle

		fprintf (resultStream,"%i %i %i %i\n",(int)box.x,(int)box.y,(int)box.width,(int)box.height);

		sprintf(strFrame, "#%d ",i) ;

        cv::putText(frame,strFrame,cvPoint(0,20),2,1,CV_RGB(25,200,25));
		
        cv::imshow("CT", frame);// Display
        cv::waitKey(30);
	}
	fclose(resultStream);
	return 0;
}
