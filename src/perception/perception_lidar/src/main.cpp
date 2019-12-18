#include <ros/ros.h>
#include "associate/base_association.h"
#include "associate/hungarian_association.cpp"
#include "associate/max_association.h"
#include "perception_lidar.h"

#include <vector>

int main(int argc, char** argv)
{
  cout << "start perception_lidar main: " << endl;

  ros::init(argc, argv, "perception_lidar");

  // BaseAssociation* base_association = new MaxAssociation(0.5);
  sensor_lidar::BaseAssociation* base_association = new sensor_lidar::HungarianAssociation(0.05);

  for (int i = 0; i < argc; i++)
  {
    printf("Argument %d is %s.\n", i, argv[i]);
  }

  bool is_draw = false;
  if (argc > 1 && strcmp(argv[1], "draw_bounding_box") == 0)
  {
    is_draw = true;
  }

  float cluster_Tolerance = 0.3;
  int min_cluster_size = 3;
  int max_cluster_siz = 30000;

  sensor_lidar::PerceptionLidar perception_lidar(base_association, cluster_Tolerance, min_cluster_size, max_cluster_siz,
                                                 is_draw);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  delete base_association;

  return 0;
}
