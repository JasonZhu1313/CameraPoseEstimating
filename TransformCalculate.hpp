//
//  TransformCalculate.hpp
//  KinectTracking
//
//  Created by 思宇朱 on 2017/10/19.
//  Copyright © 2017年 思宇朱. All rights reserved.
//

#ifndef TransformCalculate_hpp
#define TransformCalculate_hpp

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "imgprocess.h"
#include "ClusterAnalysis.h"
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
using namespace cv;
using namespace std;
using namespace imgprocess;

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
class TransformCalculate{
public:
    //the path of output file which save the intrinsic and extrinsic parameters;
    string path;
    //different flag represent that using different methods
    int flag;
    Mat cameraMatrix,coefDiss;
    //calibrate the camera to get the intrinsic and extrinsic parameters.
    int cameraCalibration(Mat &rotationVector,Mat &rotationMatrix, Mat &transformMatrix);
    int readParamFromFile();
    int keypointsTransformatiom(Mat keyPoints,Mat objectPoints,Mat &rotationVector,Mat &rotationMatrix, Mat &transformMatrix);
    int detectTransformByEagle(Mat &rotationVector,Mat &rotationMatrix, Mat &transformMatrix);
    int detectTransformByChessboard(Mat &rotationVector,Mat &rotationMatrix, Mat &transformMatrix);
private:
    static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
                                     const vector<vector<Point2f> >& imagePoints,
                                     const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                     const Mat& cameraMatrix, const Mat& distCoeffs,
                                     vector<float>& perViewErrors );
    static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType);
    static bool runCalibration( vector<vector<Point2f> > imagePoints,
                               Size imageSize, Size boardSize, Pattern patternType,
                               float squareSize, float aspectRatio,
                               int flags, Mat& cameraMatrix, Mat& distCoeffs,
                               vector<Mat>& rvecs, vector<Mat>& tvecs,
                               vector<float>& reprojErrs,
                               double& totalAvgErr);
    static void saveCameraParams( const string& filename,
                                 Size imageSize, Size boardSize,
                                 float squareSize, float aspectRatio, int flags,
                                 const Mat& cameraMatrix, const Mat& distCoeffs,
                                 const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                 const vector<float>& reprojErrs,
                                 const vector<vector<Point2f> >& imagePoints,
                                 double totalAvgErr );
    static bool readStringList( const string& filename, vector<string>& l );
    static bool runAndSave(const string& outputFilename,
                           const vector<vector<Point2f> >& imagePoints,
                           Size imageSize, Size boardSize, Pattern patternType, float squareSize,
                           float aspectRatio, int flags, Mat& cameraMatrix,
                           Mat& distCoeffs, bool writeExtrinsics, bool writePoints );
    
    
};
#endif /* TransformCalculate_hpp */
