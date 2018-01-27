//
//  ObjDetection.hpp
//  KinectTracking
//
//  Created by 思宇朱 on 2017/10/17.
//  Copyright © 2017年 思宇朱. All rights reserved.
//

#ifndef ObjDetection_hpp
#define ObjDetection_hpp

#include <stdio.h>
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;
using namespace std;
class ObjDetection{
private:
    //Preprocessing the image for HOG object detection
    cv::Mat deskew(Mat &img);
    
public:
    //Detect the object by using HOG feature + SVM
    void detectObjectsUsingHOG(Mat &img);
    //Detect the object by using faster R CNN
    void detectObjectsUsingFasterRCNN(Mat &img);
};

#endif /* ObjDetection_hpp */
