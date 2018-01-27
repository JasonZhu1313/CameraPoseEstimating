#include <iostream>
#include <cmath>
#include <vector>
#include <map>
#include "DataPoint.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


using namespace cv;
using namespace std;

//if the number of points in that cluster is less than the CLUSTERTHRESHOLD,drop that cluster
const double CLUSTERTHRESHOLD=0.3;
//if the number of points in that cluster is less than th MINPOINTS,drop that cluster
const int MINPOINTS=50;
class ClusterAnalysis
{
private:
	vector<DataPoint> dataSets;       //the total dataset
	unsigned int dimNum;            //the number of features of the point
	double radius;                 //the radius to consider
    double radiusFilter;            //used to filter the sparse point
	unsigned int dataNum;            //the number of data
	unsigned int minPTs;
    unsigned int minPTsFilter;
    unsigned int clusterNum;
   
    cv::Mat frame;//used to debug

	double GetDistance(DataPoint& dp1, DataPoint& dp2);
	void SetArrivalPoints(DataPoint& dp);
	void KeyPointCluster(unsigned long i, unsigned long clusterId);
public:
    void generateTenPointsNearTheMidPoint(Point2f midPoint,vector<Point2f> &targetPoints,float slope,float b);
    int getUpDownPoint(vector<Point2f> &dataPoints,vector<DataPoint> &cluster,vector<Point2f> &farestPoints);
    int restorePointer(vector<DataPoint> *pointcluster,int step);
    DataPoint* getFarestPoint(vector<DataPoint> &cluster);
    void processCluster(vector<Point2f> & landmarks,unsigned long &clusterNum);
    Mat getFrame();
    int getClusterNum();
    vector<DataPoint> getDataSet();
	ClusterAnalysis(){}
    bool drawCircle(Mat &target,vector<DataPoint> dataSets,int clusterNum);
    bool Init(cv::Mat &rScore, cv::Mat &frame,double radius, int minPTs,int radiusFilter,int minPTsFilter);
	bool DoDBSCANRecursive(vector<Point2f> &landmarks);
	bool WriteToFile(char* fileName);  
};
