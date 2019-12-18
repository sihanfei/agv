#ifndef _HUNGARIAN_H_
#define _HUNGARIAN_H_

#include <iostream>
#include <vector>
#include "sensor_object/base_object.h"
#include "associate/base_association.h"

using namespace std;

namespace sensor_lidar
{
	class HungarianAssociation:public BaseAssociation
	{
	public:
		HungarianAssociation(int cost_threshold);
		~HungarianAssociation();
		void getIncidenceMatrix(const map<uint32_t, sensor_lidar::BaseObject*>& global_map, const vector<sensor_lidar::BaseObject*>& new_obj, Eigen::MatrixXd& incidence_matrix);

	private:
		double Solve(vector<vector<double> >& DistMatrix, vector<int>& Assignment);
		void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
		void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
		void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
		void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
		void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	};
}


#endif // _HUNGARIAN_H_