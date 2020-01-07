
#ifndef WIT_DRIVER_HPP_
#define WIT_DRIVER_HPP_

#include <chrono>
#include <cstdlib>
#include <ecl/devices.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/threads.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
//#include "command.hpp"
#include "data.hpp"
#include "packet_handler/packet_finder.hpp"
#include "parameter.hpp"

using namespace std;
using namespace ecl;
namespace wit {

/*****************************************************************************
** Parent Interface
*****************************************************************************/
// 串口数据解码程序，使用有限状态机原理
class PacketFinder : public PacketFinderBase {
 public:
  virtual ~PacketFinder() {}
  //重定义校验函数
  bool checkSum();
};

/*****************************************************************************
 ** Interface [wit]
 *****************************************************************************/
/**
 * @brief  The core wit driver class.
 *
 * This connects to the outside world via sigslots and get accessors.
 **/
class WitDriver {
 public:
  WitDriver();
  ~WitDriver();

  //初始化
  void init(Parameter &param);
  //  检查是否连接上通信串口
  bool isConnected() { return connected_; }
  //  关闭接口
  void shutdown() { shutdown_requested_ = true; }
  bool isShutdown() const { return shutdown_requested_; }
  double getRelatedYaw() const { return relate_yaw_; }

  bool isAlive() { return alive_; }
  void resetYawOffset() { yaw_offset_ = data_.imugps_.rpy[2]; }
  //底盘数据流控制程序，运行于thread线程中
  void spin();

  //  传感器数据锁
  void lockDataAccess();
  void unlockDataAccess();

  /******************************************
  ** Getters - Raw Data Api
  *******************************************/
  /* Be sure to lock/unlock the data access (lockDataAccess and
   * unlockDataAccess)
   * around any getXXX calls - see the doxygen notes for lockDataAccess. */
  //  舵机状态信息
  Data::IMUGPS getData() const { return data_.imugps_; }

 private:
  bool connected_;
  bool alive_;

  //读取返回数据线程
  ecl::Thread thread_spin_;

  bool shutdown_requested_;
  /*********************
  ** Driver Paramters
  **********************/
  Parameter param_;

  /*********************
  ** Packet Handling
  **********************/
  Data data_;
  double relate_yaw_, yaw_offset_;

  //  下位机串口与串口buffer
  ecl::Serial serial;
  PacketFinder packet_finder;
  PacketFinder::BufferType data_buffer;
  ecl::Mutex data_mutex_;

  /*********************
  ** Signals
  **********************/
  ecl::Signal<> sig_stream_data;
};
}

#endif
