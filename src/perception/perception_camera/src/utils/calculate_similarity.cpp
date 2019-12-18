#include "utils/calculate_similarity.h"
#include <iostream>

// 计算IOU
float sensor_camera::calculateIOU(Eigen::VectorXf new_object_state, Eigen::VectorXf global_object_state)
{
    float x1_max = new_object_state(2);
    float y1_max = new_object_state(3);
    float x1_min = new_object_state(0);
    float y1_min = new_object_state(1);

    float x2_max = global_object_state(2);
    float y2_max = global_object_state(3);
    float x2_min = global_object_state(0);
    float y2_min = global_object_state(1);

    // 重叠区域坐标
    float x_max = std::min(x1_max, x2_max);
    float y_max = std::min(y1_max, y2_max);
    float x_min = std::max(x1_min, x2_min);
    float y_min = std::max(y1_min, y2_min);

    // 求交集面积
    float intersection_area = 0;
    if ((x_max > x_min) && (y_max > y_min))
    {
        intersection_area = (x_max - x_min) * (y_max - y_min);
    }

    // 求并集面积
    float union_area = (x1_max - x1_min) * (y1_max - y1_min) + (x2_max - x2_min) * (y2_max - y2_min) - intersection_area;
    if(union_area < 0.01)
    {
        return 0;
    }
    
    return intersection_area / union_area;
}

// 速度相似度
float sensor_camera::calculateVelocitySimilarity(vector<float>& new_vx_list, vector<float>& new_vy_list, vector<float>& old_vx_list, vector<float>& old_vy_list)
{
    float mean_new_vx = float(accumulate(new_vx_list.begin(), new_vx_list.end(), 0.0)) / new_vx_list.size();
    float mean_new_vy = float(accumulate(new_vy_list.begin(), new_vy_list.end(), 0.0)) / new_vy_list.size();
    float mean_old_vx = float(accumulate(old_vx_list.begin(), old_vx_list.end(), 0.0)) / old_vx_list.size();
    float mean_old_vy = float(accumulate(old_vy_list.begin(), old_vy_list.end(), 0.0)) / old_vy_list.size();

    float model_old_v = sqrt(pow(mean_old_vx, 2) + pow(mean_old_vy, 2));
    float model_new_v = sqrt(pow(mean_new_vx, 2) + pow(mean_new_vy, 2));
    float model_new_minus_old = sqrt(pow((mean_new_vx - mean_old_vx), 2) + pow((mean_new_vy - mean_old_vy), 2));

    float result = 0.0;

    if(model_old_v > 0.15)
    {
        result = 1.0 - model_new_minus_old / model_old_v;
    }
    else
    {
        result = model_new_v;
    }

    return result;
}