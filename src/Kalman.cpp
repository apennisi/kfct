/*
 *  Kalman Filter Compressive Tracking (KFCT)
 *  Copyright 2017 Andrea Pennisi
 *
 *  This file is part of AT and it is distributed under the terms of the
 *  GNU Lesser General Public License (Lesser GPL)
 *
 *
 *
 *  KFCT is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  KFCT is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with EAT.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  KFCT has been written by Andrea Pennisi
 *
 *  Please, report suggestions/comments/bugs to
 *  andrea.pennisi@gmail.com
 *
 */

#include "Kalman.h"

using namespace MyKalmanFilter;

KalmanFilter::KalmanFilter(const int &_x, const int &_y, const float &dt)
{
    KF = cv::KalmanFilter(4, 2, 0);
    state = cv::Mat_<float>(4, 1);
    processNoise = cv::Mat(4, 1, CV_32F);
    measurement = cv::Mat_<float>(2, 1);
    measurement.setTo(cv::Scalar(0));

    KF.statePre.at<float>(0, 0) = _x;
    KF.statePre.at<float>(1, 0) = _y;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;

    KF.statePost.at<float>(0) = _x;
    KF.statePost.at<float>(1) = _y;
    KF.statePost.at<float>(2) = 0;
    KF.statePost.at<float>(3) = 0;

    KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1,0,dt,0,
                                                     0,1,0,dt,
                                                     0,0,1,0,
                                                     0,0,0,1);

    cv::setIdentity(KF.measurementMatrix);
    //cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(10000));

    KF.processNoiseCov=(cv::Mat_<float>(4, 4) <<
            pow(dt,4.0)/4.0,    0,  pow(dt,3.0)/2.0,    0,
            0,  pow(dt,4.0)/4.0,   0,  pow(dt,3.0)/2.0,
            pow(dt,3.0)/2.0,    0,  pow(dt,2.0),    0,
            0,  pow(dt,3.0)/2.0,  0,    pow(dt,2.0));

    KF.processNoiseCov*=.5;
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(.1));
}

cv::Point KalmanFilter::predict()
{
    cv::Mat prediction = KF.predict();
    cv::Point predictPt(prediction.at<float>(0), prediction.at<float>(1));

    KF.statePre.copyTo(KF.statePost);
    KF.errorCovPre.copyTo(KF.errorCovPost);

    return predictPt;
}

cv::Point KalmanFilter::correct(const int &_x, const int &_y)
{
    measurement(0) = _x;
    measurement(1) = _y;
    cv::Mat estimated = KF.correct(measurement);
    cv::Point statePt(estimated.at<float>(0), estimated.at<float>(1));
    return statePt;
}
