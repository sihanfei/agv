#ifndef K_MEANS_H_
#define K_MEANS_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <limits>

using namespace std;

typedef struct Point{
	float x;
	float y;
	int cluster;
	Point (){}
	Point (float a,float b,int c){
		x = a;
		y = b;
		cluster = c;
	}
}point_;

float stringToFloat(string i);

vector<point_> openFile(const char* dataset);

float squareDistance(point_ a,point_ b);

void k_means(vector<point_> dataset,int k);



#endif