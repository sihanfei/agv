#include "utils/calculate_similarity.h"

// 计算IOU
float calculateIOU(Eigen::VectorXf new_object_state, Eigen::VectorXf global_object_state)
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
    
    return intersection_area / union_area;
}

// 速度相似度
float calculateVelocitySimilarity(float new_vx, float new_vy, float old_vx, float old_vy)
{

    float model_old_v = sqrt(pow(old_vx, 2) + pow(old_vy, 2));
    float model_new_v = sqrt(pow(new_vx, 2) + pow(new_vy, 2));
    float model_new_minus_old = sqrt(pow((new_vx - old_vx), 2) + pow((new_vy - old_vy), 2));

    float result = 0.0;

    if(model_old_v > 1.5)
    {
        result = 1.0 - model_new_minus_old / model_old_v;
    }
    else
    {
        result = 0.0;
    }

    return result;
}

float calculateVariance(vector<float>& resultSet)
{
    float variance = 0.0;
    if(resultSet.size() >= 2)
    {
        float sum = accumulate(begin(resultSet), end(resultSet), 0);
        float mean =  sum / resultSet.size(); //均值
    
        float accum  = 0.0;
        for_each (begin(resultSet), end(resultSet), [&](const float d) {
            accum  += (d - mean) * (d - mean);
        });
    
        variance = accum/(resultSet.size()-1);
    }

    return variance;
}