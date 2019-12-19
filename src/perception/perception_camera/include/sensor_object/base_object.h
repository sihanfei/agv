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

// #define DEBUG_BASEOBJECT

namespace sensor_camera
{
    class BaseObject
    {
    public:
        explicit BaseObject(
            ros::Time timestamp,
            uint8_t object_class,
            float exist_confidence,
            float class_confidence,
            sensor_camera::BaseFilter* world_filter,
            sensor_camera::BaseFilter* pix_filter,
            float state_3d[6]
        );

        virtual ~BaseObject() = 0;

        uint8_t getObjectClass() const;
        float getExistConfidence() const;
        float getClassConfidence() const;
        std::vector<float>& getSxList();
        std::vector<float>& getSyList();
        std::vector<float>& getVxList();
        std::vector<float>& getVyList();
        std::vector<float>& getDetalTList();

        virtual void setSxList(float detal_sx);
        virtual void setSyList(float detal_sy);
        virtual void setVxList(float detal_vx);
        virtual void setVyList(float detal_vy);
        virtual void setDetalTList(float detal_t);
        virtual void setObjectClass(uint8_t object_class);
        virtual void setExistConfidence(float exist_confidence);
        virtual void setClassConfidence(float class_confidence);
        void setState3d(float state_3d[6]);
        void setWorldState(Eigen::VectorXf& world_state);
        
        float calculateSimilarity(sensor_camera::BaseObject& obj);

        virtual void update(sensor_camera::BaseObject& obj) = 0;
        virtual void updatePixFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t) = 0;
        virtual void updateWorldFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t) = 0;
        virtual Eigen::VectorXf getPixState() const = 0;
        virtual Eigen::MatrixXf getPixMeasurementCov() const = 0;
        virtual Eigen::VectorXf getWorldState() const = 0;
        virtual Eigen::MatrixXf getWorldMeasurementCov() const = 0;
        virtual Eigen::VectorXf prediction(ros::Time timestamp) = 0;
        Eigen::VectorXf getState3d();

    public:
        ros::Time timestamp_;

    protected:
        uint8_t object_class_;
        float exist_confidence_;
        float class_confidence_;

        sensor_camera::BaseFilter* pix_filter_;
        sensor_camera::BaseFilter* world_filter_;

        Eigen::VectorXf state_3d_;

        std::vector<float> sx_list_;
        std::vector<float> sy_list_;
        std::vector<float> vx_list_;
        std::vector<float> vy_list_;
        std::vector<float> detal_t_list_;
        const int erase_num = 10;
    };
}
#endif // _BASEOBJECT_H_