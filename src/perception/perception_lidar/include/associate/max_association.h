#ifndef _MAXASSOCIATION_H_
#define _MAXASSOCIATION_H_

#include <Eigen/Dense>
#include "sensor_object/base_object.h"
#include "associate/base_association.h"
using namespace std;

#define DEBUG_MAX_ASSOCIATION

namespace sensor_lidar
{
    class MaxAssociation:public BaseAssociation
    {
    public:
        explicit MaxAssociation(int cost_threshold);
        ~MaxAssociation();
        void getIncidenceMatrix(const map<uint32_t, sensor_lidar::BaseObject*>& global_map, const vector<sensor_lidar::BaseObject*>& new_obj, Eigen::MatrixXd& incidence_matrix);
    };
}

#endif // _MAXASSOCIATION_H_