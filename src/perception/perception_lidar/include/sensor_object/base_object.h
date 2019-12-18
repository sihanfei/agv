#ifndef _BASEOBJECT_H_
#define _BASEOBJECT_H_

#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "filter/base_filter.h"
#include "utils/calculate_similarity.h"
#include "utils/calculate_velocity.h"

using namespace std;

#define DEBUG_BASEOBJECT
namespace sensor_lidar
{
    class BaseObject
    {
    public:
        explicit BaseObject(
        const ros::Time& timestamp,
        uint8_t object_class,
        float exist_confidence,
        float class_confidence,
        sensor_lidar::BaseFilter* filter,
        float x1,
        float y1,
        float x2,
        float y2,
        float x3,
        float y3,
        float x4,
        float y4
        );

        virtual ~BaseObject();

        uint8_t getObjectClass() const;
        float getExistConfidence() const;
        float getClassConfidence() const;
        std::vector<float>& getSxMinList();
        std::vector<float>& getSyMinList();
        std::vector<float>& getSxMaxList();
        std::vector<float>& getSyMaxList();
        std::vector<float>& getVxMinList();
        std::vector<float>& getVyMinList();
        std::vector<float>& getVxMaxList();
        std::vector<float>& getVyMaxList();
        std::vector<float>& getDetalTList();

        virtual void setSxMinList(float detal_sxmin);
        virtual void setSyMinList(float detal_symin);
        virtual void setSxMaxList(float detal_sxmax);
        virtual void setSyMaxList(float detal_symax);
        virtual void setVxMinList(float detal_vxmin);
        virtual void setVyMinList(float detal_vymin);
        virtual void setVxMaxList(float detal_vxmax);
        virtual void setVyMaxList(float detal_vymax);
        virtual void setDetalTList(float detal_t);
        virtual void setObjectClass(uint8_t object_class);
        virtual void setExistConfidence(float exist_confidence);
        virtual void setClassConfidence(float class_confidence);
        
        float calculateSimilarity(sensor_lidar::BaseObject& obj);

        virtual void update(sensor_lidar::BaseObject& obj) = 0;
        virtual void updateFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t) = 0;
        virtual Eigen::VectorXf getState() const = 0;
        virtual Eigen::MatrixXf getMeasurementCov() const = 0;
        virtual Eigen::VectorXf prediction(ros::Time timestamp) = 0;

    public:
        ros::Time timestamp_;
        // 4个点的坐标
        float x1_;
        float y1_;
        float x2_;
        float y2_;
        float x3_;
        float y3_;
        float x4_;
        float y4_;

    protected:
        uint8_t object_class_;
        float exist_confidence_;
        float class_confidence_;

        sensor_lidar::BaseFilter* filter_;

        std::vector<float> sxmin_list_;
        std::vector<float> symin_list_;
        std::vector<float> vxmin_list_;
        std::vector<float> vymin_list_;
        std::vector<float> sxmax_list_;
        std::vector<float> symax_list_;
        std::vector<float> vxmax_list_;
        std::vector<float> vymax_list_;
        std::vector<float> detal_t_list_;
        const int erase_num = 100;
    };
}
#endif // _BASEOBJECT_H_
