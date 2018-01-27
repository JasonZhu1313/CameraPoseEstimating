#include <vector>

using namespace std;

const int DIME_NUM = 2;


class DataPoint
{
private:
	unsigned long dpID;
	double dimension[DIME_NUM];
	long clusterId;
	bool isKey;
	bool visited;
	vector<unsigned long> arrivalPoints;
    vector<unsigned long> toloratePoints;
public:
	DataPoint();
	DataPoint(unsigned long dpID, double* dimension, bool isKey);

	unsigned long GetDpId();
	void SetDpId(unsigned long dpID);
	double* GetDimension();
	void SetDimension(double* dimension);
	bool IsKey();
	void SetKey(bool isKey);
	bool isVisited();                       
	void SetVisited(bool visited);
	long GetClusterId();
	void SetClusterId(long classId);
	vector<unsigned long>& GetArrivalPoints();
    vector<unsigned long>& GetToloratePoints();
};
