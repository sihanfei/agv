#include "associate/max_association.h"

sensor_camera::MaxAssociation::MaxAssociation(int cost_threshold)
    :sensor_camera::BaseAssociation(cost_threshold)
{
}

sensor_camera::MaxAssociation::~MaxAssociation()
{
}

void sensor_camera::MaxAssociation::getIncidenceMatrix(const map<uint32_t, sensor_camera::BaseObject*>& global_map, const vector<sensor_camera::BaseObject*>& new_obj, Eigen::MatrixXd& incidence_matrix)
{
#ifdef DEBUG_MAX_ASSOCIATION
    cout << "Max Association: start" << endl;
#endif

    int rows = new_obj.size();
    int cols = global_map.size();

#ifdef DEBUG_MAX_ASSOCIATION
    cout << "all_rows: " << rows << ", all_cols: " << cols << endl;
#endif

    Eigen::MatrixXf matrix_0;
    Eigen::MatrixXd matrix_1, matrix_2;
    matrix_0 = Eigen::MatrixXf::Zero(rows, cols);
    matrix_1 = Eigen::MatrixXd::Zero(rows, cols);
    matrix_2 = Eigen::MatrixXd::Zero(rows, cols);
    
    map<uint32_t, sensor_camera::BaseObject*>::const_iterator it;

    // 计算IOU匹配
    for(int i = 0; i < rows; i++)
    {
        int j = 0;
        for(it = global_map.begin(); it != global_map.end(); it++)
        {

            float cost = new_obj[i]->calculateSimilarity(*(it->second));

#ifdef DEBUG_MAX_ASSOCIATION
            cout << "row: " << i << ", cols: " << j << ", cost: " << cost << endl;
#endif

            if(cost > cost_threshold_)
            {
                matrix_0(i,j) = cost;
            }
            else
            {
                matrix_0(i,j) = 0.0;
            }
            
            j++;
        }
    }
    // 将matrix_1中的每行的最大IOU位置置1
    for(int i = 0; i < matrix_0.rows(); i++)
    {
        Eigen::MatrixXd::Index index;
        matrix_0.row(i).maxCoeff(&index);
        matrix_1(i, index) = 1;
    }
    // 将matrix_2中的每列的最大IOU位置置1
    for(int j = 0; j < matrix_0.cols(); j++)
    {
        Eigen::MatrixXd::Index index;
        matrix_0.col(j).maxCoeff(&index);
        matrix_2(index, j) = 1;
    }
    // 计算关联矩阵
    incidence_matrix = matrix_1.array() * matrix_2.array();

#ifdef DEBUG_MAX_ASSOCIATION
    cout << "Max Association: end" << endl;
#endif

}
