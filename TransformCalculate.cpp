//
//  TransformCalculate.cpp
//  KinectTracking
//
//  Created by 思宇朱 on 2017/10/19.
//  Copyright © 2017年 思宇朱. All rights reserved.
//

#include "TransformCalculate.hpp"
int TransformCalculate::detectTransformByChessboard(cv::Mat &rotationVector, cv::Mat &rotationMatrix, cv::Mat &transformMatrix){
    VideoCapture cap(0);
    if(!cap.isOpened()){
        return -1;
    }
    
    this->path="/Users/siyuzhu/Desktop/programoutput/out_camera_data.yml";
    if(this->readParamFromFile()<0){
        cout<<"the path of the parameters is wrong,you need to calibration the camera first"<<endl;
        this->cameraCalibration(rotationVector, rotationMatrix, transformMatrix);
    }
    
    Mat objectPoints= (Mat_<float>(4,3) << 0,0,0, 0,12,0, 0,7,7, 0,7,0);
    namedWindow("video");
    clock_t delay,now;
    delay=clock();
    while(1){
        now=clock();
        Mat frame,resultBinary,rScoreMat;
        cap >> frame;
        int timeInterval=20000;
        cout<<now-delay<<endl;
        if(now-delay>timeInterval){
            delay=now;
            int detectedCorner;
            vector<Point2f> landmarks;
            imgprocess::detectHarrisCorners(frame, resultBinary,rScoreMat,imgprocess::ALPHA);
            //imgprocess::drawHarrisCorners(frame, resultBinary,detectedCorner);
            ClusterAnalysis myClusterAnalysis;
            myClusterAnalysis.Init(resultBinary,frame,imgprocess::RADIUS, imgprocess::MINPTS,imgprocess::RADIUSFILTER,imgprocess::MINPTSFILTER);
            myClusterAnalysis.DoDBSCANRecursive(landmarks);
            Mat key=Mat(landmarks);
            myClusterAnalysis.drawCircle(frame,myClusterAnalysis.getDataSet(),myClusterAnalysis.getClusterNum());
            this->keypointsTransformatiom(key,objectPoints,rotationVector,rotationMatrix,transformMatrix);
            cout<<"rotationVector"<<rotationVector<<endl;
            cout<<"rotationMatrix"<<rotationMatrix<<endl;
            cout<<"transformMatrix"<<transformMatrix<<endl;
            cout<<"detected corner number of size: "<<frame.size <<" is "<<detectedCorner<<endl;
        }
    
  }

}
    
    

int TransformCalculate::detectTransformByEagle(Mat &rotationVector,Mat &rotationMatrix, Mat &transformMatrix){
    VideoCapture cap(0);
    if(!cap.isOpened()){
        return -1;
    }
    
    this->path="/Users/siyuzhu/Desktop/programoutput/out_camera_data.yml";
    
    if(this->readParamFromFile()<0){
        cout<<"the path of the parameters is wrong,you need to calibration the camera first"<<endl;
        this->cameraCalibration(rotationVector, rotationMatrix, transformMatrix);
    }
    
    Mat objectPoints= (Mat_<float>(4,3) << -5,0,0, 5,0,0, 0,0,0, 0,8,0);
    namedWindow("video");
    clock_t delay,now;
    delay=clock();
    while(1){
        now=clock();
        Mat frame,resultBinary,rScoreMat;
        cap >> frame;
        int timeInterval=20000;
        cout<<now-delay<<endl;
        if(now-delay>timeInterval){
            delay=now;
            int detectedCorner;
            vector<Point2f> landmarks;
            imgprocess::detectHarrisCorners(frame, resultBinary,rScoreMat,imgprocess::ALPHA);
            //imgprocess::drawHarrisCorners(frame, resultBinary,detectedCorner);
            ClusterAnalysis myClusterAnalysis;
            myClusterAnalysis.Init(resultBinary,frame,imgprocess::RADIUS, imgprocess::MINPTS,imgprocess::RADIUSFILTER,imgprocess::MINPTSFILTER);
            myClusterAnalysis.DoDBSCANRecursive(landmarks);
            Mat key=Mat(landmarks);
            myClusterAnalysis.drawCircle(frame,myClusterAnalysis.getDataSet(),myClusterAnalysis.getClusterNum());
            this->keypointsTransformatiom(key,objectPoints,rotationVector,rotationMatrix,transformMatrix);
            cout<<"rotationVector"<<rotationVector<<endl;
            cout<<"rotationMatrix"<<rotationMatrix<<endl;
            cout<<"transformMatrix"<<transformMatrix<<endl;
            cout<<"detected corner number of size: "<<frame.size <<" is "<<detectedCorner<<endl;
        }
        imshow("video", frame);
        if(cvWaitKey(27)=='g'){
            break;
        }
    }
    destroyWindow("video");
    return 1;
}



//using four points to solve a p4p problem to get the transform and rotation matrix.
int TransformCalculate::keypointsTransformatiom(Mat keyPoints,Mat objectPoints,Mat &rotationVector,Mat &rotationMatrix, Mat &transformMatrix){
    if(keyPoints.cols!=2&&keyPoints.rows!=4){
        cout<<"something wrong with the keypoints"<<endl;
        return -1;
    }
    if(objectPoints.cols!=2&&objectPoints.rows!=4){
        cout<<"something wrong with the keypoints"<<endl;
        return -1;
    }
    cv::solvePnP(objectPoints, keyPoints, this->cameraMatrix,this->coefDiss, rotationVector, transformMatrix);
    //convert from rotation vector to rotation matrix
    Rodrigues(rotationVector, rotationMatrix);
    return 1;
}

int TransformCalculate::readParamFromFile(){
    if(this->path==""){
        cout<<"please enter a file name"<<endl;
        return -1;
    }
     FileStorage fs(path, FileStorage::READ);
    fs["camera_matrix"] >> this->cameraMatrix;
    fs["distortion_coefficients"] >> this->coefDiss;
    if(this->cameraMatrix.empty()||this->coefDiss.empty()){
        cout<<"nothing be read"<<endl;
        return -1;
    }
    return 1;
}
int TransformCalculate::cameraCalibration( Mat &rotationVector,Mat &rotationMatrix, Mat &transformMatrix){
    Size boardSize, imageSize;
    float squareSize, aspectRatio;
    Mat cameraMatrix, distCoeffs;
    string outputFilename;
    string inputFilename = "";
    
    int i, nframes;
    bool writeExtrinsics, writePoints;
    bool undistortImage = false;
    int flags = 0;
    VideoCapture capture;
    bool flipVertical;
    bool showUndistorted;
    bool videofile;
    int delay;
    clock_t prevTimestamp = 0;
    int mode = DETECTION;
    int cameraId = 0;
    vector<vector<Point2f> > imagePoints;
    vector<string> imageList;
    Pattern pattern = CHESSBOARD;
    
    boardSize.width = 9;
    boardSize.height = 6;
   
    squareSize = 5;
    nframes = 10;
    aspectRatio =1;
    delay =1000;
    flags |= CALIB_FIX_ASPECT_RATIO;
    showUndistorted = 0;
    cameraId = 0;
    writeExtrinsics=0;
    writePoints=0;
    outputFilename=this->path;
    //outputFilename="/Users/siyuzhu/Desktop/out_camera_data.yml";
    if( !inputFilename.empty() )
    {
        if( !videofile && readStringList(inputFilename, imageList) )
            mode = CAPTURING;
        else
            capture.open(inputFilename);
    }
    else
        capture.open(cameraId);
    
   
    if( !imageList.empty() )
        nframes = (int)imageList.size();
    
    if( capture.isOpened() )
       cout<<"open Capture"<<endl;
    
    namedWindow( "Image View", 1 );
    
    for(i = 0;;i++)
    {
        Mat view, viewGray;
        bool blink = true;
        
        if( capture.isOpened() )
        {
            Mat view0;
            capture >> view0;
            view0.copyTo(view);
        }
        else if( i < (int)imageList.size() )
            view = imread(imageList[i], 1);
        
        if(view.empty())
        {
            if( imagePoints.size() > 0 )
                runAndSave(outputFilename, imagePoints, imageSize,
                           boardSize, pattern, squareSize, aspectRatio,
                           flags, cameraMatrix, distCoeffs,
                           writeExtrinsics, writePoints);
            break;
        }
        
        imageSize = view.size();
        flipVertical=0;
        if( flipVertical )
            flip( view, view, 0 );
        
        vector<Point2f> pointbuf;
        cvtColor(view, viewGray, COLOR_BGR2GRAY);
        
        bool found;
        switch( pattern )
        {
            case CHESSBOARD:
                found = findChessboardCorners( view, boardSize, pointbuf,
                                              CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
                //cout<<"--->found"<<found<<endl;
                break;
            case CIRCLES_GRID:
                found = findCirclesGrid( view, boardSize, pointbuf );
                break;
            case ASYMMETRIC_CIRCLES_GRID:
                found = findCirclesGrid( view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID );
                break;
            default:
                cout<<"error"<<endl;
        }
        
        // improve the found corners' coordinate accuracy
        if( pattern == CHESSBOARD && found) cornerSubPix( viewGray, pointbuf, Size(11,11),
                                                         Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
        
        if( mode == CAPTURING && found &&
           (!capture.isOpened() || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) )
        {
            imagePoints.push_back(pointbuf);
            prevTimestamp = clock();
            blink = capture.isOpened();
        }
        
        if(found){
             drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
             //cout<<"draw chessboardCorners"<<endl;
        }
        
        
        string msg = mode == CAPTURING ? "100/100" :
        mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);
        
        if( mode == CAPTURING )
        {
            if(undistortImage)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), nframes );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), nframes );
        }
        
        putText( view, msg, textOrigin, 1, 1,
                mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

        
        if( mode == CALIBRATED && undistortImage )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }
        
        imshow("Image View", view);
        char key = (char)waitKey(capture.isOpened() ? 50 : 500);
        
        if( key == 27 )
            break;
        
        
        if(key == 's' && mode == CALIBRATED){
            this->cameraMatrix = cameraMatrix.clone();
            this->coefDiss = distCoeffs.clone();
            
            //select 4 points
            vector<Point2f> keyPoints;
            keyPoints.push_back(pointbuf[0]);
            keyPoints.push_back(pointbuf[8]);
            keyPoints.push_back(pointbuf[45]);
            keyPoints.push_back(pointbuf[53]);
            Mat matKey=Mat(keyPoints);
            
            Mat objectPoints= (Mat_ <float>(4,3) <<0,25,0, 40,25,0, 0,0,0, 40,0,0);
            keypointsTransformatiom(matKey,objectPoints,rotationVector,rotationMatrix,transformMatrix);
            cout<<"rotationVector"<<rotationVector<<endl;
            cout<<"rotationMatrix"<<rotationMatrix<<endl;
            cout<<"transformMatrix"<<transformMatrix<<endl;
        }
        
      
        
        if( capture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
        
        if( mode == CAPTURING && imagePoints.size() >= (unsigned)nframes )
        {
            if( runAndSave(outputFilename, imagePoints, imageSize,
                           boardSize, pattern, squareSize, aspectRatio,
                           flags, cameraMatrix, distCoeffs,
                           writeExtrinsics, writePoints))
                mode = CALIBRATED;
            else
                mode = DETECTION;
            if( !capture.isOpened() )
                break;
        }
        destroyWindow("Image View");
    }   
    return -1;
}


double TransformCalculate::computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                                     const vector<vector<Point2f> >& imagePoints,
                                                     const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                                     const Mat& cameraMatrix, const Mat& distCoeffs,
                                                     vector<float>& perViewErrors ){
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());
    
    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }
    return std::sqrt(totalErr/totalPoints);
}

void TransformCalculate::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType){
    corners.resize(0);
    
    switch(patternType)
    {
        case CHESSBOARD:
        case CIRCLES_GRID:
            for( int i = 0; i < boardSize.height; i++ )
                for( int j = 0; j < boardSize.width; j++ )
                    corners.push_back(Point3f(float(j*squareSize),
                                              float(i*squareSize), 0));
            break;
            
        case ASYMMETRIC_CIRCLES_GRID:
            for( int i = 0; i < boardSize.height; i++ )
                for( int j = 0; j < boardSize.width; j++ )
                    corners.push_back(Point3f(float((2*j + i % 2)*squareSize),
                                              float(i*squareSize), 0));
            break;
            
        default:
            CV_Error(Error::StsBadArg, "Unknown pattern type\n");
    }
}

bool TransformCalculate::runCalibration(vector<vector<Point2f> > imagePoints,
                                        Size imageSize, Size boardSize, Pattern patternType,
                                        float squareSize, float aspectRatio,
                                        int flags, Mat& cameraMatrix, Mat& distCoeffs,
                                        vector<Mat>& rvecs, vector<Mat>& tvecs,
                                        vector<float>& reprojErrs,
                                        double& totalAvgErr){
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( flags & CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = aspectRatio;
    
    distCoeffs = Mat::zeros(8, 1, CV_64F);
    
    vector<vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0],CHESSBOARD);
    
    objectPoints.resize(imagePoints.size(),objectPoints[0]);
    
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, flags|CALIB_FIX_K4|CALIB_FIX_K5);
    ///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
    printf("RMS error reported by calibrateCamera: %g\n", rms);
    
    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
    
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
    
    return ok;
    
}

void TransformCalculate::saveCameraParams(const string& filename,
                                          Size imageSize, Size boardSize,
                                          float squareSize, float aspectRatio, int flags,
                                          const Mat& cameraMatrix, const Mat& distCoeffs,
                                          const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                          const vector<float>& reprojErrs,
                                          const vector<vector<Point2f> >& imagePoints,
                                          double totalAvgErr ){
    FileStorage fs( filename, FileStorage::WRITE );
    
    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );
    
    fs << "calibration_time" << buf;
    
    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;
    
    if( flags & CALIB_FIX_ASPECT_RATIO )
        fs << "aspectRatio" << aspectRatio;
    
    if( flags != 0 )
    {
        sprintf( buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
        //cvWriteComment( *fs, buf, 0 );
    }
    
    fs << "flags" << flags;
    
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    
    fs << "avg_reprojection_error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);
    
    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));
            
            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }
    
    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
    
}

bool TransformCalculate:: readStringList( const string& filename, vector<string>& l ){
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

bool TransformCalculate::runAndSave(const string& outputFilename,
                                    const vector<vector<Point2f> >& imagePoints,
                                    Size imageSize, Size boardSize, Pattern patternType, float squareSize,
                                    float aspectRatio, int flags, Mat& cameraMatrix,
                                    Mat& distCoeffs, bool writeExtrinsics, bool writePoints ){
    
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    
    bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
                             aspectRatio, flags, cameraMatrix, distCoeffs,
                             rvecs, tvecs, reprojErrs, totalAvgErr);
    printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);
    
    if( ok )
        saveCameraParams( outputFilename, imageSize,
                         boardSize, squareSize, aspectRatio,
                         flags, cameraMatrix, distCoeffs,
                         writeExtrinsics ? rvecs : vector<Mat>(),
                         writeExtrinsics ? tvecs : vector<Mat>(),
                         writeExtrinsics ? reprojErrs : vector<float>(),
                         writePoints ? imagePoints : vector<vector<Point2f> >(),
                         totalAvgErr );
    return ok;
}
