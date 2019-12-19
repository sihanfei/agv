#ifndef _BASEASSOCIATION_H_
#define _BASEASSOCIATION_H_

#include <Eigen/Dense>
#include <iostream>
#include <map>
#include "sensor_object/base_object.h"

namespace sensor_camera
{
    class BaseAssociation
    {
    public:
        explicit BaseAssociation(int cost_threshold);
        virtual ~BaseAssociation() = 0;
        virtual void getIncidenceMatrix(const std::map<uint32_t, sensor_camera::BaseObject*>& global_map, const std::vector<sensor_camera::BaseObject*>& new_obj, Eigen::MatrixXd& incidence_matrix) = 0;

    protected:
        int cost_threshold_;
    };
}

#endif // _BASEASSOCIATION_H_