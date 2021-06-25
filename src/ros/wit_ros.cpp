#include "../../include/wit_node/wit_ros.hpp"
#include <float.h>
#include <tf/tf.h>
#include <ecl/streams/string_stream.hpp>
#include <string>

namespace wit {

WitRos::WitRos(string &node_name)
    : name_(node_name), slot_stream_data_(&WitRos::processStreamData, *this) {}

WitRos::~WitRos() {
  ROS_INFO_STREAM("Wit: waiting for wd_ thread finish[" << name_ << " ].");
}

bool WitRos::init(NodeHandle &nh) {
  advertiseTopics(nh);
  subscribeTopics(nh);
  slot_stream_data_.connect(name_ + string("/stream_data"));

  Parameter parameter;

  parameter.ns = name_;

  if (!nh.getParam("port", parameter.port_)) {
    ROS_ERROR_STREAM(
        "Wit : no wit device port given on the parameter server (e.g. "
        "/dev/ttyUSB0)["
        << name_ << "].");
    return false;
  }

  /*********************
   ** Driver Init
   **********************/
  try {
    wd_.init(parameter);
    ros::Duration(0.1).sleep();
  } catch (const ecl::StandardException &e) {
    switch (e.flag()) {
      case (ecl::OpenError): {
        ROS_ERROR_STREAM("Wit : could not open connection ["
                         << parameter.port_ << "][" << name_ << "].");
        break;
      }
      default: {
        ROS_ERROR_STREAM("Wit : initialisation failed [" << name_ << "].");
        ROS_DEBUG_STREAM(e.what());
        break;
      }
    }
    return false;
  }

  return true;
}

bool WitRos::update() {
  if (wd_.isShutdown()) {
    ROS_ERROR_STREAM("Wit : Driver has been shutdown. Stopping update loop. ["
                     << name_ << "].");
    return false;
  }
  if (!wd_.isConnected()) {
    ROS_ERROR_STREAM(
        "Wit : arm serial port is not connetced, please connect the arm.");
    return false;
  }
  processStreamData();
  return true;
}

void WitRos::advertiseTopics(NodeHandle &nh) {
  imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu", 1000);
  gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("/gps", 1000);
  raw_data_pub_ = nh.advertise<wit_node::ImuGpsRaw>("raw_data", 1000);
  related_yaw_pub_ = nh.advertise<std_msgs::Float64>("related_yaw", 1000);
}

void WitRos::subscribeTopics(NodeHandle &nh) {
  reset_offset_sub_ = nh.subscribe(string("reset_offset"), 10,
                                   &WitRos::subscribeResetOffset, this);
}
void WitRos::subscribeResetOffset(const std_msgs::Empty msg) {
  wd_.resetYawOffset();
}

void WitRos::processStreamData() {
  if (ok()) {
    sensor_msgs::Imu imu_msg;
    sensor_msgs::NavSatFix gps_msg;
    wit_node::ImuGpsRaw raw_msg;
    std_msgs::Float64 yaw_msg;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = ros::Time::now();
    gps_msg.header.stamp = ros::Time::now();
    raw_msg.header.stamp = ros::Time::now();
    Data::IMUGPS data = wd_.getData();
    imu_msg.angular_velocity.x = data.w[0];
    imu_msg.angular_velocity.y = data.w[1];
    imu_msg.angular_velocity.z = data.w[2];

    imu_msg.linear_acceleration.x = data.a[0];
    imu_msg.linear_acceleration.y = data.a[1];
    imu_msg.linear_acceleration.z = data.a[2];

    imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(
        data.rpy[0], data.rpy[1], data.rpy[2]);

    imu_msg.orientation_covariance =
    {0.001, 0.0,    0.0,
     0.0,   0.001,  0.0,
     0.0,   0.0,    0.001};
    imu_msg.angular_velocity_covariance =
    {0.00001, 0.0,      0.0,
     0.0,     0.00001,  0.0,
     0.0,     0.0,      0.00001};
    imu_msg.linear_acceleration_covariance =
    {0.01,  0.0,  0.0,
     0.0,   0.01, 0.0,
     0.0,   0.0,  0.01};

    gps_msg.altitude = data.altitude;
    gps_msg.latitude = data.latitude;
    gps_msg.longitude = data.longtitude;
    gps_msg.position_covariance_type =
        sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    for (int i = 0; i < 3; i++) {
      raw_msg.acc.push_back(data.a[i]);
      raw_msg.gyro.push_back(data.w[i]);
      raw_msg.rpy.push_back(data.rpy[i]);
      raw_msg.mag.push_back(data.mag[i]);
      raw_msg.dop.push_back(data.gpsa[i]);
    }
    for (int i = 0; i < 4; i++) {
      raw_msg.ps.push_back(data.d[i]);
      raw_msg.quarternion.push_back(data.q[i]);
    }
    raw_msg.sn = data.satelites;
    raw_msg.gpsh = data.gpsh;
    raw_msg.gpsy = data.gpsy;
    raw_msg.gpsv = data.gpsv;
    raw_msg.ap = data.pressure;
    raw_msg.longtitude = data.longtitude;
    raw_msg.altitude = data.altitude;
    raw_msg.latitude = data.latitude;
    raw_msg.time = data.timestamp;
    raw_msg.temperature = data.temperature;

    yaw_msg.data = wd_.getRelatedYaw();

    imu_pub_.publish(imu_msg);
    gps_pub_.publish(gps_msg);
    raw_data_pub_.publish(raw_msg);
    related_yaw_pub_.publish(yaw_msg);
  }
}
}
