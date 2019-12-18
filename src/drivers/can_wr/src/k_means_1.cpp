#include "k_means_1.h"
using namespace std;

#define INT_MAX 65535

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

float stringToFloat(string i){
	stringstream sf;
	float score=0;
	sf<<i;
	sf>>score;
	return score;
}
vector<point_> openFile(const char* dataset){
	fstream file;
	file.open(dataset,ios::in);
	vector<point_> data;
	while(!file.eof()){
		string temp;
		file>>temp;
		int split = temp.find(',',0);
		point_ p(stringToFloat(temp.substr(0,split)),stringToFloat(temp.substr(split+1,temp.length()-1)),0);
		data.push_back(p);
	}		
	file.close();
	return data;
}
float squareDistance(point_ a,point_ b){
	return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}
void k_means(vector<point_> dataset,int k){
	vector<point_> centroid;
	int n=1;
	int len = dataset.size();
	srand((int)time(0));
	//random select centroids
	while(n<=k){
		int cen = (float)rand()/(RAND_MAX+1)*len;
		point_ cp(dataset[cen].x,dataset[cen].y,n);
		centroid.push_back(cp);
		n++;
	}
	for(int i=0;i<k;i++){
		cout<<"x:"<<centroid[i].x<<"\ty:"<<centroid[i].y<<"\tc:"<<centroid[i].cluster<<endl;
	}
	//cluster
	int time = 100;
	int oSSE = INT_MAX;
	int nSSE = 0;
	while(abs(oSSE-nSSE)>=1){
//	while(time){
		oSSE = nSSE;
		nSSE = 0;
		//update cluster for all the points
		for(int i=0;i<len;i++){
			n=1;
			float shortest = INT_MAX;
			int cur = dataset[i].cluster;
			while(n<=k){
				float temp=squareDistance(dataset[i],centroid[n-1]);			
				if(temp<shortest){
					shortest = temp;
					cur = n;
				}
				n++;
			}
			dataset[i].cluster = cur;
		}
		//update cluster centroids
		int *cs = new int[k];
		for(int i=0;i<k;i++) cs[i] = 0;
		for(int i=0;i<k;i++){
			centroid[i] = point_(0,0,i+1);
		}
		for(int i=0;i<len;i++){
			centroid[dataset[i].cluster-1].x += dataset[i].x;
			centroid[dataset[i].cluster-1].y += dataset[i].y;
			cs[dataset[i].cluster-1]++;
		}
		for(int i=0;i<k;i++){
			centroid[i].x /= cs[i];
			centroid[i].y /= cs[i];
		}
		cout<<"time:"<<time<<endl;
		for(int i=0;i<k;i++){
			cout<<"x:"<<centroid[i].x<<"\ty:"<<centroid[i].y<<"\tc:"<<centroid[i].cluster<<endl;
		}	
		//SSE
		for(int i=0;i<len;i++){
			nSSE += squareDistance(centroid[dataset[i].cluster-1],dataset[i]);
		}
//		time--;
	}
	fstream clustering;
	clustering.open("clustering.txt",ios::out);
	for(int i=0;i<len;i++){
		clustering<<dataset[i].x<<","<<dataset[i].y<<","<<dataset[i].cluster<<"\n";
	}
	clustering.close();
//	cout<<endl;
//	for(int i=0;i<centroid.size();i++){
//		cout<<"x:"<<centroid[i].x<<"\ty:"<<centroid[i].y<<"\tc:"<<centroid[i].cluster<<endl;
//	}
}

