#include "ClusterAnalysis.h"
#include <fstream>
#include <iosfwd>

#include <math.h>



Mat ClusterAnalysis::getFrame(){
    return this->frame;
}
bool ClusterAnalysis::Init(cv::Mat &rScoreBinary,cv::Mat &rScoreValue, double radius, int minPTs,int radiusFilter,int minPTsFilter)
{
    this->frame=rScoreValue;
	this->radius = radius;
	this->minPTs = minPTs;
    this->minPTsFilter=minPTsFilter;
    this->radiusFilter=radiusFilter;
	this->dimNum = DIME_NUM;
	unsigned long count=0;
    Mat_<uchar>::const_iterator it= rScoreBinary.begin<uchar>();
    Mat_<uchar>::const_iterator itd=rScoreBinary.end<uchar>();
    
    for(int i=0;it!=itd;i++,it++){
        if(*it){
            DataPoint tempDP;
            //DIME_NUM=2,we only consider the distance
            double tempDimData[DIME_NUM];
            tempDimData[0]=(i%rScoreBinary.cols);
            tempDimData[1]=(i/rScoreBinary.cols);
            tempDP.SetDimension(tempDimData);
            tempDP.SetDpId(count);
            count++;
            tempDP.SetVisited(false);
            tempDP.SetClusterId(-1);
            dataSets.push_back(tempDP);
        }
    }
	dataNum = count;
	cout << "the total number of data is "<< dataNum<< endl;
	for (unsigned long i = 0; i<dataNum; i++)
	{
		SetArrivalPoints(dataSets[i]);
	}
	return true;
}


bool ClusterAnalysis::drawCircle(Mat &target,vector<DataPoint> dataSets,int clusterNum){
    Scalar color[10]={
        Scalar(255,0,255),Scalar(255,0,0),Scalar(103,38,158), Scalar(200,200,169), Scalar(121,175,155),
        Scalar(250,218,241),Scalar(252,157,154),Scalar(249,205,173), Scalar(179,168,150), Scalar(222,125,44)
    };
    unsigned long i;
    for(i=0;i<dataNum;i++){
        if(dataSets[i].GetClusterId()>=0){
            //cout<<"点坐标 x:"<<dataSets[i].GetDimension()[0]<<", y:" << dataSets[i].GetDimension()[1]<< endl;
            circle(target, Point(dataSets[i].GetDimension()[0],dataSets[i].GetDimension()[1]), 3, color[dataSets[i].GetClusterId()%10], 1);
        }
    }
    return true;
}


int ClusterAnalysis::getClusterNum(){
    return clusterNum;
}


//You can write the reslut to the file
bool ClusterAnalysis::WriteToFile(char* fileName)
{
	ofstream of1(fileName);
	for (unsigned long i = 0; i<dataNum; i++)
	{
		for (int d = 0; d<DIME_NUM; d++)
			of1 << dataSets[i].GetDimension()[d] << '\t';
		of1 << dataSets[i].GetClusterId() << endl;
	}
	of1.close();
	return true;
}


void ClusterAnalysis::SetArrivalPoints(DataPoint& dp)
{
	for (unsigned long i = 0; i<dataNum; i++)
	{
		double distance = GetDistance(dataSets[i], dp);
        if(distance <= radiusFilter && i!=dp.GetDpId()){
            dp.GetToloratePoints().push_back(i);
        }
        if (distance <= radius && i != dp.GetDpId()){
            dp.GetArrivalPoints().push_back(i);
        }
	}
    //cout<<"tolorate points"<<dp.GetToloratePoints().size()<<endl;
    if(dp.GetToloratePoints().size()<=minPTsFilter){
        dataSets.pop_back();
        dataNum--;
    }
    //cout<<"arrival points"<<dp.GetArrivalPoints().size()<<endl;
	if (dp.GetArrivalPoints().size() >= minPTs)
	{
		dp.SetKey(true);
		return;
	}
	dp.SetKey(false);
}

bool ClusterAnalysis::DoDBSCANRecursive(vector<Point2f> &landmarks)
{
	unsigned long clusterId = 0;
	for (unsigned long i = 0; i<dataNum; i++)
	{
        
		DataPoint& dp = dataSets[i];
		if ( !dp.isVisited() &&dp.IsKey())
		{
			dp.SetClusterId(clusterId);
			dp.SetVisited(true);
			KeyPointCluster(i, clusterId);
			clusterId++;
		}
	}
    processCluster(landmarks,clusterId);
    this->clusterNum=clusterId;
	cout << "the number of cluster = " << clusterId << endl;
	return true;
}

int ClusterAnalysis::restorePointer(vector<DataPoint> *pointCluster, int step){
    for(int i=0;i<step;i++){
        if((--pointCluster)==nullptr){
            cout<<"something error with the restore process";
            return -1;
        }
    }
    return 1;
}


int ClusterAnalysis::getUpDownPoint(vector<Point2f> &dataPoints,vector<DataPoint> &cluster,vector<Point2f> &farestPoints){
    if(farestPoints.size()!=2){
        return -1;
    }
    float resulta=0;
    float resultb=0;
    Point2f pf1=farestPoints[0];
    Point2f pf2=farestPoints[1];
    float maxdis=0;
    float mindis=0;
    float midx=(pf1.x+pf2.x)/2;
    float midy=(pf1.y+pf2.y)/2;
    
    if((pf2.x-pf1.x)==0){
        vector<Point2f> targetPoints;
        targetPoints.push_back(Point2f(midx,midy));
        for(int i=1;i<=2;i++){
            targetPoints.push_back(Point2f(midx+i,midy));
            targetPoints.push_back(Point2f(midx-i,midy));
        }
        float dis=0;
        for(int i=0;i<targetPoints.size();i++){
            for(int j =0;j<cluster.size();j++){
                float x=cluster[j].GetDimension()[0];
                float y= cluster[j].GetDimension()[1];
                
                dis= y-targetPoints[i].y;
                if(x-targetPoints[i].x<3&&x-targetPoints[i].x>-3){
                    if(dis>maxdis){
                        maxdis=dis;
                        resulta=x;
                        resultb=y;
                    }
                }
             
            }
        }
        if(maxdis!=0){
            dataPoints.push_back(Point2f(resulta,resultb));
            dataPoints.push_back(Point2f(resulta,midy));
            cout<<"down point"<< resulta<<","<<resultb<<endl;
            return 1;
        }
       
    }else{
        float slope=(pf2.y-pf1.y)/(pf2.x-pf1.x);
        
        float b =midy-midx*slope;
        
        cout<<"slope"<<slope<<endl;
        cout<<"b"<<b<<endl;
        
        
        vector<Point2f> targetPoints;
        targetPoints.push_back(Point2f(midx,midy));
        for(int i=1;i<=5;i++){
            targetPoints.push_back(Point2f(midx+1*i,(midx+1*i)*slope+b));
            targetPoints.push_back(Point2f(midx-1*i,(midx-1*i)*slope+b));
        }
        for(int i=0;i<targetPoints.size();i++){
            DataPoint p;
            double tempDimData[DIME_NUM];
            tempDimData[0]=(targetPoints[i].x);
            tempDimData[1]=(targetPoints[i].y);
            p.SetDimension(tempDimData);
            for(int j =0;j<cluster.size();j++){
                
                float x=cluster[j].GetDimension()[0];
                float y= cluster[j].GetDimension()[1];
                if(x!=p.GetDimension()[0]&&y!=p.GetDimension()[1]){
                    if((-1/slope)*x+(p.GetDimension()[1]+(p.GetDimension()[0]/slope))-y< 1){
                        float dis=GetDistance(p, cluster[j]);
                        if(dis>maxdis){
                            maxdis=dis;
                            resulta=x;
                            resultb=y;
                        }
                    }
                }
            }
        }
        if(maxdis!=0){
            dataPoints.push_back(Point2f(resulta,resultb));
            float x1=((resultb-(-1/slope)*resulta)-b)/(slope+1/slope);
            float y1=slope *x1+b;
            dataPoints.push_back(Point2f(midx,midy));
            return 1;
        }
    }
    return -1;
}


//filter the dataset which is too small(noise) and get the cluster of our own target
void ClusterAnalysis::processCluster(vector<Point2f> &landmarks,unsigned long & clusterNum){
    vector<Point2f> fourDataPoints;
    vector<vector<DataPoint>> processedCluster;
    int clusterMemberCount[clusterNum];
    //initialize the processCluster
    for(int i=0;i<clusterNum;i++){
        vector<DataPoint> cluster;
        processedCluster.push_back(cluster);
        clusterMemberCount[i]=0;
    }
    //if it has at least one cluster, process the clusters
    if(clusterNum > 0){
         int count =0;
        for (int i=0;i<dataSets.size();i++){
            unsigned long id=dataSets[i].GetClusterId();
            int cluster=0;
            if(id!=-1){
                count++;
                //add the cluster to the cluster vector in the cluster array
                //cout<<"点ID"<<dataSets[i].GetClusterId()<<endl;
                processedCluster[id].push_back(dataSets[i]);
                //after adding the clusterTemp pointer has to be restore
                clusterMemberCount[id]++;
                
            }
        }
        for(int i=0;i<clusterNum;i++){
            //cout<<"cluster id: "<<i<<" has "<<clusterMemberCount[i]<<"points"<<endl;
            //if it passes, get the farest points in that cluster.
            if(clusterMemberCount[i]>CLUSTERTHRESHOLD*count&&clusterMemberCount[i]>60){
                DataPoint * point;
                //get the cluster
                //check the farest points in a cluster
                point = getFarestPoint(processedCluster[i]);
                vector<Point2f> farestVector;
                if(point!=nullptr){
                    cout<<"The cluster id = "<< (*point).GetClusterId()<<", The coordinate of the two points are:"<<endl;
                    double x1=(*point).GetDimension()[0];
                    double y1=(*point).GetDimension()[1];
                    farestVector.push_back(Point2f(x1,y1));
                    fourDataPoints.push_back(Point2f(x1,y1));
                    double x2=(*(point+1)).GetDimension()[0] ;
                    double y2=(*(point+1)).GetDimension()[1];
                    fourDataPoints.push_back(Point2f(x2,y2));
                    farestVector.push_back(Point2f(x2,y2));
                    
                    vector<Point2f> upandDown;
                    getUpDownPoint(upandDown, processedCluster[i], farestVector);
                    if(!upandDown.empty()){
                        fourDataPoints.push_back(upandDown[0]);
                        fourDataPoints.push_back(upandDown[1]);
                    }else{
                        continue;
                    }
                    
                    landmarks = fourDataPoints;
                    
                    
                    cout<<"A : ("<< x1 <<","<<y1<< ")"<<endl;
                    cout<<"B : ("<< x2<<","<<y2 << ")"<<endl;
                    Point p1;
                    Point p2;
                    for(int i=0;i<landmarks.size();i++){
                        cout<<"point "<<i<<" = "<<landmarks[i].x<<","<<landmarks[i].y<<endl;
                        if(i!=2){
                             cv::circle(this->frame,  Point(landmarks[i].x,landmarks[i].y),3, Scalar(0,255,0),6);
                        }else{
                            cv::circle(this->frame,  Point(landmarks[i].x,landmarks[i].y),10, Scalar(255,255,0),6);
                        }
                       
                    }
                    
                    p1.x=landmarks[0].x+15;
                    p1.y=landmarks[0].y-0.8*(landmarks[2].y-landmarks[3].y);
                    p2.x=landmarks[1].x-15;
                    p2.y=landmarks[2].y+15;
                    
                    rectangle(this->frame, p1, p2,Scalar(0,0,255),4);
                    
                    cv::line(this->frame, Point(x1,y1), Point(x2,y2), Scalar(255,0,0),4,7);
                    
                }
            }
        }
        
    }
//
//    vector<DataPoint> *reslut=NULL;
//    //if it has at least one cluster, process the clusters
//    if(clusterNum > 0){
//        vector<DataPoint>* clusterArray=NULL;
//        //used for storing the initial location of the clusterArray
//        vector<DataPoint> * clusterTemp=NULL;
//        int clusterMemberCount[clusterNum];
//        int countStep=0;
//         for(int i=0;i<clusterNum;i++){
//             vector<DataPoint> temp;
//             clusterMemberCount[i]=0;
//             clusterArray=&temp;
//             clusterArray++;
//             //used to calculate how long the step goes
//             countStep++;
//             if(i==0){
//                 clusterTemp=clusterArray;
//             }
//         }
//        cout<<clusterNum<<endl;
//
//        reslut=clusterArray;
//        //used to count all the cluster points in the image
//        int count =0;
//        for (int i=0;i<dataSets.size();i++){
//            unsigned long id=dataSets[i].GetClusterId();
//            if(id!=-1){
//                count++;
//                cout<<clusterMemberCount[id]<<endl;
//                //add the cluster to the cluster vector in the cluster array
//                cout<<"点ID"<<dataSets[i].GetClusterId()<<endl;
//                DataPoint datapoint=dataSets[i];
//                (*(clusterArray+id)).push_back(datapoint);
//                restorePointer(clusterArray, id);
//                //after adding the clusterTemp pointer has to be restore
//                clusterMemberCount[id]++;
//
//            }
//        }
    
    }



DataPoint* ClusterAnalysis::getFarestPoint(vector<DataPoint> &cluster){
    int maxDistance=0;
    int a=0;
    int b=0;
    for(int i=0;i<cluster.size();i++){
        for(int j=0;j<i;j++){
            if(i!=j){
                double distance=GetDistance(cluster[i], cluster[j]);
                if(distance>maxDistance){
                    maxDistance=distance;
                    //cout<<"----> maxDistance "<< maxDistance<<endl;
                    a=i;b=j;
                    
                }
            }
        }
    }
    cout<<"the index of first point is "<<a<<endl;
    cout<<"the index of the second point is "<< b<<endl;
    cout<<"tht first point "<< cluster[a].GetDimension()[0]<<","<<cluster[a].GetDimension()[1]<<endl;
    cout<<"the second point"<< cluster[b].GetDimension()[0]<<","<<cluster[b].GetDimension()[1]<<endl;
    DataPoint* points=new DataPoint[2];
    points[0]=cluster[a];
    points[1]=cluster[b];
    return points;
}


void ClusterAnalysis::KeyPointCluster(unsigned long dpID, unsigned long clusterId)
{
	DataPoint& srcDp = dataSets[dpID];
	vector<unsigned long>& arrvalPoints = srcDp.GetArrivalPoints();
    unsigned long i;
	for (i = 0; i<arrvalPoints.size(); i++)
	{
        //search the indirect access point
		DataPoint& desDp = dataSets[arrvalPoints[i]];
		if (!desDp.isVisited())
		{
			desDp.SetClusterId(clusterId);
			desDp.SetVisited(true);
			if (desDp.IsKey())
			{
//                cout<<"目标点 （"<<srcDp.GetDimension()[0] << "," << srcDp.GetDimension()[1]<<") 的周围key Point ("<<desDp.GetDimension()[0] << "," << desDp.GetDimension()[1]<<")"<<endl;
//                cout<<"---------> 连接，src的id "<<srcDp.GetClusterId()<<" des的id "<<desDp.GetClusterId()<<endl;
//                if(desDp.GetDimension()[0]==797&&desDp.GetDimension()[1]==323){
//                     cout<<"the src point of the cluster"<<srcDp.GetClusterId()<<srcDp.GetDimension()[0]<<","<<srcDp.GetDimension()[1]<<endl;
//                    cout<<"the key point of the cluster"<<desDp.GetClusterId()<<desDp.GetDimension()[0]<<","<<desDp.GetDimension()[1]<<endl;
//                }
//                if(desDp.GetDimension()[0]==797&&desDp.GetDimension()[1]==323){
//                    cout<<"the src point of the cluster"<<srcDp.GetClusterId()<<srcDp.GetDimension()[0]<<","<<srcDp.GetDimension()[1]<<endl;
//                    cout<<"the key point of the cluster"<<desDp.GetClusterId()<<desDp.GetDimension()[0]<<","<<desDp.GetDimension()[1]<<endl;
//                }
              //  cout<<"the key point of the cluster"<<desDp.GetClusterId()<<desDp.GetDimension()[0]<<","<<desDp.GetDimension()[1]<<endl;
                //circle(frame,Point(desDp.GetDimension()[0],desDp.GetDimension()[1]), 20, Scalar(0,0,255));
                 // cout<<"---------> src 的id"<<srcDp.GetDpId()<<endl;
                //cout<<"---------> desp 的id"<<desDp.GetDpId()<<endl;
				KeyPointCluster(desDp.GetDpId(), clusterId);
			}
		}
	}
}
double ClusterAnalysis::GetDistance(DataPoint& dp1, DataPoint& dp2)
{
	double distance = 0;
	for (int i = 0; i<DIME_NUM; i++)
	{
		distance += pow(dp1.GetDimension()[i] - dp2.GetDimension()[i], 2);
	}
	return pow(distance, 0.5);
}

vector<DataPoint> ClusterAnalysis::getDataSet(){
    return dataSets;
}

