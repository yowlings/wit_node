
#include "../../include/wit_node/command.hpp"

namespace wit {

const unsigned char Command::header0 = 0xFF;
const unsigned char Command::header1 = 0xFF;
const uint16_t ALL_SPEED[14] = {10,   1000, 1000, 1000, 1000, 20, 20,
                                1000, 1000, 1000, 1000, 20,   20, 10};

//读取单个舵机的错误状态信息指令发送
Command Command::setSingleMotorErrorRead(const uint8_t &index) {
  Command outgoing;
  outgoing.data.index = index;
  outgoing.data.command = Command::Ping;
  return outgoing;
}

//控制单个舵机指令发送
Command Command::setSingleMotorControl(const uint8_t &index,
                                       const uint16_t &location) {
  Command outgoing;
  outgoing.data.index = index;
  outgoing.data.location = location;
  outgoing.data.command = Command::Write;
  return outgoing;
}

//读取单个舵机的所有状态信息指令发送
Command Command::setSingleMotorStatusRead(const uint8_t &index) {
  Command outgoing;
  outgoing.data.index = index;
  outgoing.data.read_start = 0x38;
  outgoing.data.read_length = 15;
  outgoing.data.command = Command::Read;
  return outgoing;
}

//编号为1~12的手臂关节舵机控制
Command Command::setSycnMotorControl(const uint16_t *sycn_location) {
  Command outgoing;
  outgoing.data.index = 0xFE;
  for (int i = 0; i < 12; i++) {
    outgoing.data.sycn_location[i] = sycn_location[i];
  }
  outgoing.data.command = Command::SycnWrite;
  return outgoing;
}

//编号为7~12的左手关节舵机位置控制
Command Command::setLeftArmControl(const uint16_t *left_arm_location) {
  Command outgoing;
  outgoing.data.index = 0xFE;
  for (int i = 0; i < 6; i++) {
    outgoing.data.left_arm_location[i] = left_arm_location[i];
  }
  outgoing.data.command = Command::LeftArm;
  return outgoing;
}

//编号为1~6的右手手臂关节舵机控制
Command Command::setRightArmControl(const uint16_t *right_arm_location) {
  Command outgoing;
  outgoing.data.index = 0xFE;
  for (int i = 0; i < 6; i++) {
    outgoing.data.right_arm_location[i] = right_arm_location[i];
  }
  outgoing.data.command = Command::RightArm;
  return outgoing;
}

/*****************************************************************************
** Implementation [Serialisation]
*****************************************************************************/
/**
 * 重置缓存区
 */
void Command::resetBuffer(Buffer &buffer) {
  buffer.clear();
  buffer.resize(256);
  buffer.push_back(Command::header0);
  buffer.push_back(Command::header1);
}

bool Command::serialise(ecl::PushAndPop<unsigned char> &byteStream) {
  // need to be sure we don't pass through an emum to the Trans'd buildBytes
  // functions.
  unsigned char cmd = static_cast<unsigned char>(data.command);
  uint8_t size_byte;
  uint8_t index;
  //  uint16_t speed;
  uint16_t time;
  uint8_t address;
  uint8_t label1;
  uint8_t label2;
  uint8_t sycn_cmd(0x83);
  switch (data.command) {
    case Ping:
      buildBytes(data.index, byteStream);
      size_byte = 0x02;
      buildBytes(size_byte, byteStream);
      buildBytes(cmd, byteStream);
      break;
    case Read:
      buildBytes(data.index, byteStream);
      size_byte = 0x04;
      buildBytes(size_byte, byteStream);
      buildBytes(cmd, byteStream);
      buildBytes(data.read_start, byteStream);
      buildBytes(data.read_length, byteStream);
      break;
    case Write:
      buildBytes(data.index, byteStream);
      size_byte = 0x09;
      buildBytes(size_byte, byteStream);
      buildBytes(cmd, byteStream);
      address = 0x2A;
      buildBytes(address, byteStream);
      buildBytes(data.location, byteStream);
      time = 0;
      buildBytes(time, byteStream);
      buildBytes(ALL_SPEED[data.index], byteStream);
      break;
    case RegWrite:
      buildBytes(cmd, byteStream);
      break;
    case Action:
      index = 0xFE;
      buildBytes(index, byteStream);
      size_byte = 0x02;
      buildBytes(size_byte, byteStream);
      buildBytes(cmd, byteStream);
      break;
    case SycnWrite:
      buildBytes(data.index, byteStream);
      size_byte = 0x58;
      buildBytes(size_byte, byteStream);
      buildBytes(cmd, byteStream);
      label1 = 0x2A;
      buildBytes(label1, byteStream);
      label2 = 0x06;
      buildBytes(label2, byteStream);
      for (uint8_t i = 1; i < 13; i++) {
        buildBytes(i, byteStream);
        buildBytes(data.sycn_location[i - 1], byteStream);
        time = 0;
        buildBytes(time, byteStream);
        buildBytes(ALL_SPEED[i], byteStream);
      }
      break;
    case LeftArm:
      buildBytes(data.index, byteStream);
      size_byte = 0x2E;
      buildBytes(size_byte, byteStream);
      buildBytes(sycn_cmd, byteStream);
      label1 = 0x2A;
      buildBytes(label1, byteStream);
      label2 = 0x06;
      buildBytes(label2, byteStream);
      for (uint8_t i = 7; i < 13; i++) {
        buildBytes(i, byteStream);
        buildBytes(data.left_arm_location[i - 7], byteStream);
        time = 0;
        buildBytes(time, byteStream);
        buildBytes(ALL_SPEED[i], byteStream);
      }
      break;
    case RightArm:
      buildBytes(data.index, byteStream);
      size_byte = 0x2E;
      buildBytes(size_byte, byteStream);
      buildBytes(sycn_cmd, byteStream);
      label1 = 0x2A;
      buildBytes(label1, byteStream);
      label2 = 0x06;
      buildBytes(label2, byteStream);
      for (uint8_t i = 1; i < 7; i++) {
        buildBytes(i, byteStream);
        buildBytes(data.right_arm_location[i - 1], byteStream);

        time = 0;
        buildBytes(time, byteStream);
        buildBytes(ALL_SPEED[i], byteStream);
      }
      //    std::cout<<byteStream.size()<<std::endl;
      break;
    default:
      return false;
      break;
  }
  return true;
}

}  // namespace xbot
