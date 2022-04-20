#ifndef HECTOR_MAPPING_ROS_H__
#define HECTOR_MAPPING_ROS_H__

#include "ros/ros.h"

#include <boost/thread.hpp>

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include "sensor_msgs/LaserScan.h"
#include <std_msgs/String.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "laser_geometry/laser_geometry.h"
#include "nav_msgs/GetMap.h"

#include "slam_main/HectorSlamProcessor.h"
#include "scan/DataPointContainer.h"
#include "util/MapLockerInterface.h"
#include "util/PoseInfoContainer.h"

class MapPublisherContainer
{
public:
  ros::Publisher mapPublisher_;
  ros::Publisher mapMetadataPublisher_;
  nav_msgs::GetMap::Response map_;
  ros::ServiceServer dynamicMapServiceServer_;
};

class HectorMappingRos
{
public:
  HectorMappingRos();
  ~HectorMappingRos();
  void scanCallback(const sensor_msgs::LaserScan& scan);
  void publishMap(MapPublisherContainer& map_, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex = 0);
  void rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap);
  void setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap);
  void publishMapLoop(double p_map_pub_period_);

protected:
  int lastGetMapUpdateIndex;
  ros::NodeHandle node_;
  ros::Subscriber scanSubscriber_;
  ros::Publisher posePublisher_;
  std::vector<MapPublisherContainer> mapPubContainer;
  boost::thread* map__publish_thread_;

  hectorslam::HectorSlamProcessor* slamProcessor;
  hectorslam::DataContainer laserScanContainer;

  PoseInfoContainer poseInfoContainer_;

  //-----------------------------------------------------------
  // Parameters

  std::string p_map_frame_;
  std::string p_scan_topic_;
  int p_scan_subscriber_queue_size_;

  double p_update_factor_free_;
  double p_update_factor_occupied_;
  double p_map_update_distance_threshold_;
  double p_map_update_angle_threshold_;

  double p_map_resolution_;
  int p_map_size_;
  double p_map_start_x_;
  double p_map_start_y_;
  int p_map_multi_res_levels_;

  double p_map_pub_period_;

  float p_sqr_laser_min_dist_;
  float p_sqr_laser_max_dist_;
  float p_laser_z_min_value_;
  float p_laser_z_max_value_;
};

#endif
