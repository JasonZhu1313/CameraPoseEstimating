//
//  imgprocess.h
//  KinectTracking
//
//  Created by 思宇朱 on 2017/10/12.
//  Copyright © 2017年 思宇朱. All rights reserved.
//


#ifndef IMGPROCESS_H
#define IMGPROCESS_H
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

namespace imgprocess {
    const int RADIUS=20;
    const int MINPTS=20;
    const double ALPHA=0.03;
    const double QUANTITYLEVEL=0.03;
    const int RADIUSFILTER=100;
    const int MINPTSFILTER=500;
    void detectHarrisCorners(const Mat& imgSrc, Mat& imgDst,Mat & rScore,double alpha);
    void drawHarrisCorners(Mat &image, const Mat &binary,int &detectedCorner);
}
#endif /* imgprocess_h */
