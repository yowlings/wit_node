

#ifndef WIT_ROS_HPP
#define WIT_ROS_HPP

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <wit_node/ImuGpsRaw.h>
#include <chrono>
#include <cstdlib>
#include <ecl/sigslots.hpp>
#include <ecl/threads.hpp>
#include <fstream>
#include <iostream>
#include "wit_driver.hpp"

using namespace std;
using namespace ros;

namespace wit {
class WitRos {
 public:
  WitRos(string &node_name);
  ~WitRos();
  bool init(NodeHandle &nh);
  bool update();

 private:
  /*********************
   ** Variables
   **********************/
  string name_;
  WitDriver wd_;

  /*********************
   ** Ros Publishers
   **********************/
  void advertiseTopics(NodeHandle &nh);
  Publisher imu_pub_;
  Publisher gps_pub_;
  Publisher raw_data_pub_;
  Publisher related_yaw_pub_;

  /*********************
   ** Ros Subscribers
   **********************/
  Subscriber reset_offset_sub_;
  void subscribeTopics(NodeHandle &nh);

  void subscribeResetOffset(const std_msgs::Empty msg);

  /*********************
   ** SigSlots
   **********************/

  ecl::Slot<> slot_stream_data_;

  /*********************
   ** Slot Callbacks
   **********************/
  void processStreamData();
};
}

#endif
