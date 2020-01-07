/**
 * @file include/xbot_arm_driver/command.hpp
 *
 * @brief Command structure.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_arm_driver/LICENSE
 **/
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef WIT_COMMAND_DATA_HPP__
#define WIT_COMMAND_DATA_HPP__

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/containers.hpp>
#include "packet_handler/payload_base.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

//#define GRIP_SPEED 20
//#define MOTOR_SPEED_BIG 1000
//#define MOTOR_SPEED_TINY 20

namespace wit {

class Command : public packet_handler::payloadBase {
 public:
  typedef ecl::PushAndPop<unsigned char> Buffer;
  typedef ecl::Stencil<Buffer> BufferStencil;

  /**
   * These values are used to detect the type of sub-payload that is ensuing.
   */
  enum Name {
    Ping = 1,
    Read = 2,
    Write = 3,
    RegWrite = 4,
    Action = 5,
    Reset = 6,
    LeftArm = 7,
    RightArm = 8,
    SycnWrite = 0x83
  };

  /**
   * @brief Data structure containing data for commands.
   *
   * It is important to keep this
   * state as it will have to retain knowledge of the last known command in
   * some instances - e.g. for gp_out commands, quite often the incoming command
   * is only to set the output for a single led while keeping the rest of the
   * current
   * gp_out values as is.
   *
   * For generating individual commands we modify the data here, then copy the
   * command
   * class (avoid doing mutexes) and spin it off for sending down to the device.
   */
  struct Data {
    Data()
        : command(Ping),
          index(0),
          location(2047),
          read_start(0x38),
          read_length(15) {
      for (int i = 1; i < 13; i++) {
        sycn_location[i] = 2047;
      }
    }

    Name command;
    uint8_t index;
    uint16_t location;
    uint8_t read_start;
    uint8_t read_length;
    uint16_t sycn_location[12];
    uint16_t left_arm_location[6];
    uint16_t right_arm_location[6];
  };

  virtual ~Command() {}

  //  读取某个舵机的ERROR信息
  static Command setSingleMotorErrorRead(const uint8_t &index);

  //  控制某个舵机运动
  static Command setSingleMotorControl(const uint8_t &index,
                                       const uint16_t &location);

  //  同步控制12个舵机运动
  static Command setSycnMotorControl(const uint16_t *sycn_location);

  //  同步控制6个左手舵机运动
  static Command setLeftArmControl(const uint16_t *left_arm_location);

  //  同步控制6个右手舵机运动
  static Command setRightArmControl(const uint16_t *right_arm_location);

  //  读取某个舵机的状态信息
  static Command setSingleMotorStatusRead(const uint8_t &index);

  Data data;

  void resetBuffer(Buffer &buffer);
  bool serialise(ecl::PushAndPop<unsigned char> &byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> &byteStream) {
    return true;
  } /**< Unused **/

 private:
  static const unsigned char header0;
  static const unsigned char header1;
};

}  // namespace xbot

#endif /* WIT_COMMAND_DATA_HPP__ */
