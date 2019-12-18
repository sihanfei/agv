#ifndef __VELODYNE_POINTCLOUD_RAWDATA_H
#define __VELODYNE_POINTCLOUD_RAWDATA_H

#include <boost/format.hpp>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string>

#include "velodyne_msgs/VelodyneScan.h"
#include "velodyne_pointcloud/calibration.h"
#include "velodyne_pointcloud/datacontainerbase.h"
#include "velodyne_pointcloud/point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace velodyne_rawdata
{
typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud< VPoint > VPointCloud;

static const int SIZE_BLOCK      = 100;
static const int RAW_SCAN_SIZE   = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION   = 0.01f;
static const uint16_t ROTATION_MAX_UNITS = 36000u;
static const float DISTANCE_RESOLUTION   = 0.002f;

static const uint16_t UPPER_BANK = 0xeeff; // big-endian machines
static const uint16_t LOWER_BANK = 0xddff; // little-endian machines

static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING  = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f;
static const float VLP16_DSR_TOFFSET     = 2.304f;
static const float VLP16_FIRING_TOFFSET  = 55.296f;

typedef struct raw_block
{
  uint16_t header;
  uint16_t rotation;
  uint8_t data[BLOCK_DATA_SIZE];
} raw_block_t;

union two_bytes {
  uint16_t uint;
  uint8_t bytes[2];
};

static const int PACKET_SIZE        = 1206;
static const int BLOCKS_PER_PACKET  = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCAN_PER_PACKET    = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
} raw_packet_t;

class RawData
{
public:
  RawData();
  ~RawData()
  {
  }

  int setup(ros::NodeHandle &node);
  int setupOffline(std::string calibration_file, double max_range_, double min_range_);
  double unpack(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase &data);
  void setParameters(double min_range, double max_range, double view_direction, double view_width);

private:
  typedef struct
  {
    std::string calibrationFile;
    double max_range;
    double min_range;
    int min_angle;
    int max_angle;

    double tmp_min_angle;
    double tmp_max_angle;

    double view_direction;
    double view_width;
  } Config;
  Config config_;

  velodyne_pointcloud::Calibration calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];

  double unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase &data);
  bool pointInRange(float range)
  {
    return (range >= config_.min_range && range <= config_.max_range);
  }
};
}

#endif
