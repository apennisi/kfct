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

#ifndef KALMAN_H
#define KALMAN_H

#include <opencv2/opencv.hpp>

namespace MyKalmanFilter
{
    class KalmanFilter
    {
    public:
        KalmanFilter(const int &_x, const int &_y, const float &dt);
        cv::Point predict();
        cv::Point correct(const int &_x, const int &_y);
    private:
        //the kalman filter
        cv::KalmanFilter KF;
        cv::Mat_<float> measurement;
        cv::Mat processNoise;
        cv::Mat_<float> state;
    };

}

#endif //KALMAN_H
