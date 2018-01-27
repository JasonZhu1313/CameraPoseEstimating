//
//  ObjDetection.cpp
//  KinectTracking
//
//  Created by 思宇朱 on 2017/10/17.
//  Copyright © 2017年 思宇朱. All rights reserved.
//

#include "ObjDetection.hpp"


void ObjDetection::detectObjectsUsingHOG(Mat &img){
    HOGDescriptor hog( Size(20,20), //winSize
                      Size(10,10), //blocksize
                      Size(5,5), //blockStride,
                      Size(10,10), //cellSize,
                      9, //nbins,
                      1, //derivAper,
                      -1, //winSigma,
                      0, //histogramNormType,
                      0.2, //L2HysThresh,
                      1,//gammal correction,
                      64,//nlevels=64
                      1);//Use signed gradients
    // im is of type Mat
    vector<float> descriptors;
    // compute the feature of the image
    hog.compute(img,descriptors);
    
    
    

}

Mat ObjDetection::deskew(Mat &img){
    Moments m = moments(img);
    if(abs(m.mu02) < 1e-2)
    {
        // No deskewing needed.
        return img.clone();
    }
    // Calculate skew based on central momemts.
    double skew = m.mu11/m.mu02;
    // Calculate affine transform to correct skewness.
    Mat warpMat = (Mat_<double>(2,3) << 1, skew, -0.5*skew, 0, 1 , 0);
    
    Mat imgOut = Mat::zeros(img.rows, img.cols, img.type());
    warpAffine(img, imgOut, warpMat, imgOut.size(),cv::WARP_INVERSE_MAP |cv::INTER_LINEAR);
    
    return imgOut;
}
