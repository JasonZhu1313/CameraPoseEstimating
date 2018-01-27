#include "DataPoint.h"


DataPoint::DataPoint()
{
}


DataPoint::DataPoint(unsigned long dpID, double* dimension, bool isKey) :isKey(isKey), dpID(dpID)
{
	
	for (int i = 0; i<DIME_NUM; i++)
	{
		this->dimension[i] = dimension[i];
	}
}

vector<unsigned long>& DataPoint::GetToloratePoints(){
    return toloratePoints;
}
void DataPoint::SetDimension(double* dimension)
{
	for (int i = 0; i<DIME_NUM; i++)
	{
		this->dimension[i] = dimension[i];
	}
}


double* DataPoint::GetDimension()
{
	return this->dimension;
}


bool DataPoint::IsKey()
{
	return this->isKey;
}


void DataPoint::SetKey(bool isKey)
{
	this->isKey = isKey;
}

unsigned long DataPoint::GetDpId()
{
	return this->dpID;
}


void DataPoint::SetDpId(unsigned long dpID)
{
	this->dpID = dpID;
}


bool DataPoint::isVisited()
{
	return this->visited;
}

void DataPoint::SetVisited(bool visited)
{
	this->visited = visited;
}


long DataPoint::GetClusterId()
{
	return this->clusterId;
}

void DataPoint::SetClusterId(long clusterId)
{
	this->clusterId = clusterId;
}


vector<unsigned long>& DataPoint::GetArrivalPoints()
{
	return arrivalPoints;
}
