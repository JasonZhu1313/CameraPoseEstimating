//
//  main.cpp
//  KinectTracking
//
//  Created by 思宇朱 on 2017/9/29.
//  Copyright © 2017年 思宇朱. All rights reserved.
//


//#include "imgprocess.h"
#include "TransformCalculate.hpp"
//#include "ClusterAnalysis.h"
#include <opencv2/tracking.hpp>
#include <time.h>
#include "directlineartransform.hpp"
#include "ProjectUtil.hpp"


void combineChessBoardCol();
void combineChessBoardRow();
int invokeCameraAndDetect();
int processImage();
void detectFootball();
int trackingObject();
void invokeDlt();
void invokeProject();


void useOpencv(){
    DLT dltTool;
    string imagePointPath="/Users/siyuzhu/Public/WorkSpace/xCodeWorkspace/KinectTracking/resourse/image5.txt";
    string objectPointPath="/Users/siyuzhu/Public/WorkSpace/xCodeWorkspace/KinectTracking/resourse/3d5.txt";
    if(dltTool.readDataFromFile(imagePointPath,objectPointPath )<0){
        cout<<"error,return"<<endl;
        
    }
       dltTool.init();
    
        dltTool.calculateParam();
        Mat p=dltTool.getp();
        if(!p.empty()){
            for(int i=0;i<p.rows;i++){
                for(int j=0;j<p.cols;j++){
                    cout<<"p ("<<i<<","<<j<<") = "<<p.at<float>(i,j)<<endl;
                }
            }
        }
    
    
        std::ifstream fileread("/Users/siyuzhu/Public/WorkSpace/xCodeWorkspace/KinectTracking/resourse/3d5.txt");
        vector<Point3d> pts;
        while(!fileread.eof()){
            double x=0;
            double y=0;
            double z=0;
            fileread >> x >> y >> z;
            Point3d p(x, y ,z);
            pts.push_back(p);
        }
        vector<Point2d> imagePoints=dltTool.reprojectToColorSpace(pts);
        for(int i=0;i<imagePoints.size();i++){
            cout << "projected point( " << imagePoints[i].x<< ","<< imagePoints[i].y<< " ) "<<endl;
        }
    
        dltTool.calculateError(imagePoints);
}
void useEigen(){
    DLT dltTool;
    string imagePointPath="/Users/siyuzhu/Public/WorkSpace/xCodeWorkspace/KinectTracking/resourse/image2.txt";
    string objectPointPath="/Users/siyuzhu/Public/WorkSpace/xCodeWorkspace/KinectTracking/resourse/3d2.txt";
    if(dltTool.readDataFromFile(imagePointPath,objectPointPath )<0){
        cout<<"error,return"<<endl;
    }
    VectorXf solution= dltTool.initEigen();
    Mat p;
    p.create(3,4,CV_32FC1);
    float m1_1=1.0;
    for(int i=0;i<p.rows;i++)
    {
        for(int j=0;j<p.cols;j++)
        {
            if(i==0&&j==0)
            {
                p.at<float>(i,j)=m1_1;
            }
            else
            {
                p.at<float>(i,j)=solution[i*4+j-1];
            }
        }
    }
    Mat pline1;
    Mat pline2;
    Mat pline3;
    pline1.create(1, 4, CV_32FC1);
    pline2.create(1, 4, CV_32FC1);
    pline3.create(1, 4, CV_32FC1);
    //init the param
    for(int i=0;i<4;i++){
        pline1.at<float>(0,i)=p.at<float>(0,i);
        //cout<<"********"<<p.at<float>(0,i)<<endl;
        pline2.at<float>(0,i)=p.at<float>(1,i);
        //cout<<"++++++++"<<p.at<float>(1,i)<<endl;
        pline3.at<float>(0,i)=p.at<float>(2,i);
        //cout<<"--------"<<p.at<float>(2,i)<<endl;
    }
    
}
void invokeProject(){
    std::string filename = "/Users/siyuzhu/Public/hku_lab/mismatch/Calibrari/mismatch/Data/Depth/mydepth/depth2.png";
    Mat iriamgeraw = cv::imread(filename,CV_LOAD_IMAGE_UNCHANGED);
    Mat iriamge;
    cv::flip(iriamgeraw, iriamge, 1);
    cout<<iriamge.type()<<endl;
//    for(int i=0;i<iriamge.rows;i++){
//        for (int j =0;j<iriamge.cols;j++){
//            cout<<iriamge.at<double>(i,j)<<endl;
//        }
//    }
    ProjectUtil util;
    util.init();
   
    vector<Point3d> pointCloud;
    vector<Point3d> colorSpace;
    vector<Point2d> colorImagecor;
    util.projectToPointCloud(iriamge,pointCloud);
    util.correspondingColorCoordinates(pointCloud, colorImagecor);
    
    util.savePointCloud("/Users/siyuzhu/Public/hku_lab/mismatch/Calibrari/mismatch/Data/result/myresult11.off",pointCloud,colorImagecor);
}
int main(){
    invokeProject();
    
    
  
//    TransformCalculate t;
//    Mat rotationVector,rotationMatrix,transformMatrix;
//    int flag=0;
//    if(flag==0){
//       t.detectTransformByEagle(rotationVector,rotationMatrix,transformMatrix);
//    }else if(flag==1){
//        t.cameraCalibration(rotationVector, rotationMatrix, transformMatrix);
//    }else if(flag==2){
//        t.detectTransformByChessboard(rotationVector, rotationMatrix, transformMatrix);
//    }else if(flag==3){
//        processImage();
//    }
 
    
//    useOpencv();
//    return 1;
    
    
//    Mat rotationVector,rotationMatrix,transformMatrix;
//    t.cameraCalibration(rotationVector, rotationMatrix, transformMatrix);
    
//    Mat rotationVector,rotationMatrix,transformMatrix;
//    t.path="/Users/siyuzhu/Desktop/out_camera_data.yml";
//    //Using the camera calibration to detect the intrinsic and extrinsic parameters
//    if(t.path==""){
//        //Get the cameraMatrix and coefDiss
//        t.cameraCalibration();
//    }else{
//        //if some specific path has been defined
//        if(t.readParamFromFile()){
//            if(t.detectTransformByChessboard(rotationVector,rotationMatrix, transformMatrix)){
//                cout<<"rotationvector: "<<rotationVector<<endl;
//                cout<<"rotationmatrix: "<<rotationMatrix<<endl;
//                cout<<"transformmatrix: "<<transformMatrix<<endl;
//            }
//            if(t.detectTransformByEagle(rotationVector, rotationMatrix, transformMatrix){
//                cout<<"rotationvector: "<<rotationVector<<endl;
//                cout<<"rotationmatrix: "<<rotationMatrix<<endl;
//                cout<<"transformmatrix: "<<transformMatrix<<endl;
//            }
//        }
//    }
}
void combineChessBoardCol(){
    Mat chessboard=imread("/Users/siyuzhu/Desktop/IMG_0823.JPG");
    Mat chassboardSmall;
    resize(chessboard, chassboardSmall, Size(0,0),0.2,0.2);
    
    
    cv::Mat mergedDescriptors(chassboardSmall.rows,chassboardSmall.cols*7, chassboardSmall.type());
    cv::Mat submat = mergedDescriptors.colRange(0, chassboardSmall.cols);
    chassboardSmall.copyTo(submat);
  
    for(int j=1;j <=6;j++){
        submat = mergedDescriptors.colRange(j*chassboardSmall.cols, (j+1)*chassboardSmall.cols);
        chassboardSmall.copyTo(submat);
    }
//    int totalCols=3;
//    cv::Mat mergedDescriptors(chassboardSmall.rows,totalCols*chessboard.cols, chassboardSmall.type());
//    cv::Mat submat;
//    submat= mergedDescriptors.colRange(0, 1* chassboardSmall.cols);
//    chassboardSmall.copyTo(submat);
//    submat= mergedDescriptors.colRange(1*chassboardSmall.cols, 2* chassboardSmall.cols);
//    chassboardSmall.copyTo(submat);
//    submat= mergedDescriptors.colRange(2*chassboardSmall.cols, 3* chassboardSmall.cols);
//    chassboardSmall.copyTo(submat);
//    submat= mergedDescriptors.colRange(3*chassboardSmall.cols, 4* chassboardSmall.cols);
//    chassboardSmall.copyTo(submat);
//    submat= mergedDescriptors.colRange(4*chassboardSmall.cols, 5* chassboardSmall.cols);
 //   chassboardSmall.copyTo(submat);
//    for(int i=0;i<totalCols;i++){
//        submat= mergedDescriptors.colRange(i*chassboardSmall.cols, (i+1)* chassboardSmall.cols);
//        chassboardSmall.copyTo(submat);
//    }
    imwrite("/Users/siyuzhu/Desktop/smallchessboard3.jpg", mergedDescriptors);
    namedWindow("test");
    imshow("test", mergedDescriptors);
    cvWaitKey(0);
    
}
void detectFootball(){
    Mat ball=imread("/Users/siyuzhu/Desktop/football.jpeg");
    Mat grayBall,img;
    
    //【检测结果 向量】
    std::vector<Vec3f> circles;
    cvtColor(ball, grayBall, cv::COLOR_BGR2GRAY);
    GaussianBlur( grayBall, grayBall, Size(9, 9), 2, 2 );
    HoughCircles( grayBall, circles, CV_HOUGH_GRADIENT,  1.5, 10, 200, 100, 0, 0);
    //【复制显示】
    Mat display = ball.clone();
    cout<<circles.size()<<endl;
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));       //Center ——（x，y）
        int radius = cvRound(circles[i][2]);                                //半径——r
        cout<<circles[i][0]<<","<<circles[i][1]<<endl;
        circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );            //【中心圆】
        circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );        //【外轮廓圆】
    }
    namedWindow("ball");
    imshow("ball",display);
    cvWaitKey(0);
}


int trackingObject(){
    string trackerTypes[6] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN"};
    string trackerType=trackerTypes[4];
    Ptr<Tracker> tracker;
    if (trackerType == "BOOSTING")
        tracker = TrackerBoosting::create();
    if (trackerType == "MIL")
        tracker = TrackerMIL::create();
    if (trackerType == "KCF")
        tracker = TrackerKCF::create();
    if (trackerType == "TLD")
        tracker = TrackerTLD::create();
    if (trackerType == "MEDIANFLOW")
        tracker = TrackerMedianFlow::create();
    if (trackerType == "GOTURN")
        tracker = TrackerGOTURN::create();
    
    
    VideoCapture video(0);
    if(!video.isOpened()){
        cout<<"There is something wrong with the video"<<endl;
        return -1;
    }
    Mat frame;
    bool ok=video.read(frame);
    // Define initial boundibg box
    Rect2d bbox(287, 200, 86, 320);
    // Uncomment the line below to select a different bounding box
    //bbox = selectROI(frame, false);
    
    // Display bounding box.
    rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
    imshow("Tracking", frame);
    
    tracker->init(frame, bbox);
    while(video.read(frame)){
        // Start timer
        double timer = (double)getTickCount();
        
        // Update the tracking result
        bool ok = tracker->update(frame, bbox);
        
        // Calculate Frames per second (FPS)
        float fps = getTickFrequency() / ((double)getTickCount() - timer);
        
        if (ok)
        {
            // Tracking success : Draw the tracked object
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
        }
        else
        {
            // Tracking failure detected.
            putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        }
        
        // Display tracker type on frame
        putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);
        
        // Display FPS on frame
        putText(frame, "FPS : " + int(fps), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
        
        // Display frame.
        imshow("Tracking", frame);
        // Exit if ESC pressed.
        int k = waitKey(1);
        if(k == 27)
        {
            break;
        };
    }
    return 1;
}



void combineChessBoardRow(){
    Mat c9=imread("/Users/siyuzhu/Desktop/smallchessboard9.jpg");
    Mat c7=imread("/Users/siyuzhu/Desktop/smallchessboard7.jpg");
    Mat c5=imread("/Users/siyuzhu/Desktop/smallchessboard5.jpg");
    Mat c3=imread("/Users/siyuzhu/Desktop/smallchessboard3.jpg");
    Mat chessboard=imread("/Users/siyuzhu/Desktop/IMG_0823.JPG");
    Mat chassboardSmall;
    resize(chessboard, chassboardSmall, Size(0,0),0.05,0.05);
    
    cv::Mat mergedDescriptors(c9.rows*5,c9.cols, c9.type());
    cv::Mat submat = mergedDescriptors.rowRange(0, 1);
    c9.copyTo(submat);
    submat = mergedDescriptors.rowRange(1, 2);
    c7.copyTo(submat);
    submat = mergedDescriptors.rowRange(2, 3);
    c5.copyTo(submat);
    submat = mergedDescriptors.rowRange(3, 4);
    c3.copyTo(submat);
    submat = mergedDescriptors.rowRange(4, 5);
    chassboardSmall.copyTo(submat);
    
    namedWindow("test");
    imshow("test", mergedDescriptors);
    cvWaitKey(0);
    
    
}



int invokeCameraAndDetect(){
    VideoCapture cap(0);
    if(!cap.isOpened()){
        return -1;
    }
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
            myClusterAnalysis.Init(resultBinary,rScoreMat,imgprocess::RADIUS, imgprocess::MINPTS,imgprocess::RADIUSFILTER,imgprocess::MINPTSFILTER);
            myClusterAnalysis.DoDBSCANRecursive(landmarks);
            myClusterAnalysis.drawCircle(frame,myClusterAnalysis.getDataSet(),myClusterAnalysis.getClusterNum());
            cout<<"detected corner number of size: "<<frame.size <<" is "<<detectedCorner<<endl;
        }
        imshow("video", frame);
        if(cvWaitKey(27)=='g'){
            break;
        }
        destroyWindow("video");
    }
    return 1;
}

int processImage(){
        Mat oringin =imread("/Users/siyuzhu/Desktop/programoutput/a.jpg");
        Mat smallSize;
        TransformCalculate t;
        t.path="/Users/siyuzhu/Desktop/programoutput/out_camera_data.yml";
        Mat rotationVector,rotationMatrix,transformMatrix;
        t.readParamFromFile();
        Mat objectPoints= (Mat_ <float>(4,3) << -5,0,0, 5,0,0, 0,0,0, 0,8,0);

    
    
//        resize(oringin, smallSize, Size(0,0),0.7,0.3);
//        oringin=smallSize.clone();
    

        if(oringin.empty()){
            cout<<"the path of the picture is wrong"<<endl;
            return -1;
        }
        Mat resultOringin,rScoreMat;
        int detectedCorner;
        vector<Point2f> landmarks;
        imgprocess::detectHarrisCorners(oringin, resultOringin,rScoreMat, imgprocess::ALPHA);
        //imgprocess::drawHarrisCorners(oringin, resultOringin,detectedCorner);
        ClusterAnalysis myClusterAnalysis;
        myClusterAnalysis.Init(resultOringin, oringin, imgprocess::RADIUS, imgprocess::MINPTS,imgprocess::RADIUSFILTER,imgprocess::MINPTSFILTER);
        myClusterAnalysis.DoDBSCANRecursive(landmarks);
        Mat key=Mat(landmarks);
        myClusterAnalysis.drawCircle(oringin,myClusterAnalysis.getDataSet(),myClusterAnalysis.getClusterNum());
        t.keypointsTransformatiom(key,objectPoints,rotationVector,rotationMatrix,transformMatrix);
        cout<<"rotationVector"<<rotationVector<<endl;
        cout<<"rotationMatrix"<<rotationMatrix<<endl;
        cout<<"transformMatrix"<<transformMatrix<<endl;
        cout<<"detected corner number of size: "<<oringin.size <<" is "<<detectedCorner<<endl;
        // imwrite("/Users/siyuzhu/Desktop/result.jpg", oringin);
        namedWindow("bigimage");
        imshow("bigimage", oringin);
        cvWaitKey(0);
        return 1;
}

//int main(){
//
//    Mat oringin =imread("/Users/siyuzhu/Desktop/boywhite.jpg");
//    if(oringin.empty()){
//        return -1;
//    }
//    Mat small1;
//    Size s(0,0);
//    resize(oringin, small1,s,0.5,0.5);
//
//    Mat small2;
//    resize(small1, small2,s,0.4,0.4);
//
//
//    Size s1(100,100);
//    Mat reshap;
//    resize(oringin, reshap, s1);

//    int detectedCorner;

//    binary result
//    Mat resultOringin,resultSmall1,resultSmall2,resultReshape;
//    imgprocess::detectHarrisCorners(oringin, resultOringin, 0.01);
//    imgprocess::drawHarrisCorners(oringin, resultOringin,detectedCorner);
//    ClusterAnalysis myClusterAnalysis;
//    myClusterAnalysis.Init(resultOringin, 50, 20);
//    myClusterAnalysis.DoDBSCANRecursive();
//    myClusterAnalysis.drawCircle(oringin,myClusterAnalysis.getDataSet(),myClusterAnalysis.getClusterNum());
//    cout<<"detected corner number of size: "<<oringin.size <<" is "<<detectedCorner<<endl;
//    namedWindow("bigimage");
//    imshow("bigimage", oringin);

//    imgprocess::detectHarrisCorners(small1, resultSmall1, 0.01);
//    imgprocess::drawHarrisCorners(small1, resultSmall1,detectedCorner);
//    cout<<"detected corner number of size: "<<small1.size <<" is "<<detectedCorner<<endl;
//    namedWindow("small1image");
//    imshow("small1image", small1);
//
//    imgprocess::detectHarrisCorners(small2, resultSmall2, 0.01);
//    imgprocess::drawHarrisCorners(small2, resultSmall2,detectedCorner);
//    cout<<"detected corner number of size: "<<small2.size <<" is "<<detectedCorner<<endl;
//    namedWindow("small2image");
//    imshow("small2image", small2);
//
//
//    imgprocess::detectHarrisCorners(reshap, resultReshape, 0.01);
//    imgprocess::drawHarrisCorners(reshap, resultReshape,detectedCorner);
//    cout<<"detected corner number of size: "<<reshap.size <<" is "<<detectedCorner<<endl;
//    namedWindow("reshapImage");
//    imshow("reshapeImage", reshap);
//


//    waitKey(0);
//    destroyWindow("bigimage");
//    destroyWindow("small1image");
//    destroyWindow("small2image");
//    destroyWindow("reshapImage");

//}




