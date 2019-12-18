#include "utils/calculate_velocity.h"

pair<float, float> calculateMeanVxVy(vector<float>& sx_list, vector<float>& sy_list, vector<float>& detal_t_list)
{
    float total_t = accumulate(detal_t_list.begin(), detal_t_list.end(), 0.0);
    float mean_vx = float(accumulate(sx_list.begin(), sx_list.end(), 0.0)) / total_t;
    float mean_vy = float(accumulate(sy_list.begin(), sy_list.end(), 0.0)) / total_t;

#ifdef DEBUG_FUSIONOBJECT
    cout << "mean_vx: " << mean_vx << "  mean_vy: " << mean_vx << endl;
#endif

    return pair<float, float>(mean_vx, mean_vy);
}

float calculateVariance(vector<float>& resultSet)
{
    float variance = 0.0;
    if(resultSet.size() >= 2)
    {
        float sum = accumulate(begin(resultSet), end(resultSet), 0);
        float mean =  sum / resultSet.size(); //均值
    
        float accum  = 0.0;
        for_each(begin(resultSet), end(resultSet), [&](const float d) {
            accum  += (d - mean) * (d - mean);
        });
    
        variance = accum/(resultSet.size()-1);
    }

    return variance;
}
