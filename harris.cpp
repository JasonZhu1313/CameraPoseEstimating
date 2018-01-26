//
//  harris.cpp
//  KinectTracking
//
//  Created by 思宇朱 on 2017/10/12.
//  Copyright © 2017年 思宇朱. All rights reserved.
//


#include "imgprocess.h"

void imgprocess::detectHarrisCorners(const cv::Mat &imgSrc, cv::Mat &imgDst, cv::Mat &rScore ,double alpha){
    Mat gray;
   
    if(imgSrc.channels()==3){
        cvtColor(imgSrc, gray, CV_RGB2GRAY);
    }else{
        gray=imgSrc.clone();
    }
    gray.convertTo(gray, CV_64F);
    Mat xKernel = (Mat_<double>(1,3)<<-1,0,1);
    Mat yKernel = xKernel.t();
    
    Mat Ix,Iy;
    filter2D(gray, Ix, CV_64F, xKernel);
    filter2D(gray, Iy, CV_64F, yKernel);
    
    
    Mat Ix2,Iy2,Ixy;
    Ix2 = Ix.mul(Ix);
    Iy2 = Iy.mul(Iy);
    Ixy = Ix.mul(Iy);
    
    
    
    Mat gaussKernel = getGaussianKernel(7, 1);
    filter2D(Ix2, Ix2, CV_64F, gaussKernel);
    filter2D(Iy2, Iy2, CV_64F, gaussKernel);
    filter2D(Ixy, Ixy, CV_64F, gaussKernel);
    
    Mat cornerStrength(gray.size(), gray.type());
    for(int i=0;i<gray.rows;i++){
        for(int j =0;j<gray.cols;j++){
            double det_m = Ix2.at<double>(i,j) * Iy2.at<double>(i,j) - Ixy.at<double>(i,j) * Ixy.at<double>(i,j);
            
            double trace_m = Ix2.at<double>(i,j) + Iy2.at<double>(i,j);
            
            double rScore= det_m - alpha * trace_m *trace_m;
            cornerStrength.at<double>(i,j) = rScore;
        }
    }
   
    // threshold,the maxmium value in the local area
    double maxStrength,minStrength;
    Point minPoint, maxPoint;
    minMaxLoc(cornerStrength, &minStrength, &maxStrength, &minPoint, &maxPoint);
    Mat dilated;
    Mat localMax;
    dilate(cornerStrength, dilated, Mat());
    compare(cornerStrength, dilated, localMax, CMP_EQ);
    
    //the maxmium should larger than the
    Mat cornerMap;
    double qualityLevel = 0.01;
    double thresh = qualityLevel * maxStrength;
    cornerMap = cornerStrength > thresh;
    rScore=cornerMap.clone();
    //true:255 otherwise:0
    bitwise_and(cornerMap, localMax, cornerMap);
    
    imgDst = cornerMap.clone();

}

void imgprocess:: drawHarrisCorners(cv::Mat &image, const cv::Mat &binary,int &detectedCorner){
    Mat_<uchar>::const_iterator it= binary.begin<uchar>();
    Mat_<uchar>::const_iterator itd=binary.end<uchar>();
    int count = 0;
    for (int i = 0; it != itd; it++, i++)
    {
        if (*it){
            circle(image, Point(i%image.cols, i / image.cols), 3, Scalar(0, 255, 0), 1);
            count++;
        }
    }
    detectedCorner=count;
}
