/**
 * @file /xbot_arm_driver/src/driver/core_sensors.cpp
 *
 * @brief Implementation of the core sensor packet data.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_arm_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/wit_node/data.hpp"
#include <time.h>
#include "../../include/wit_node/packet_handler/payload_headers.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace wit {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool Data::serialise(ecl::PushAndPop<unsigned char> &byteStream) {
  return true;
}

bool Data::desTime(ecl::PushAndPop<unsigned char> &byteStream) {
  uint8_t tmpL, tmpH;
  uint16_t tmp;
  buildVariable(tmpL, byteStream);  // 20YY
                                    //  std::cout << int(tmpL) << std::endl;
  buildVariable(tmpL, byteStream);  // MM
  buildVariable(tmpL, byteStream);  // DD
  imugps_.timestamp = tmpL * 24 * 60 * 60.0;
  buildVariable(tmpL, byteStream);  // HH
  imugps_.timestamp += tmpL * 60 * 60.0;
  buildVariable(tmpL, byteStream);  // MM
  imugps_.timestamp += tmpL * 60.0;
  buildVariable(tmpL, byteStream);  // SS
  imugps_.timestamp += tmpL;
  buildVariable(tmp, byteStream);  // MSL,MSH
  imugps_.timestamp += tmp / 1000.0;
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.timestamp << std::endl;
    return true;
  } else {
    return false;
  }
}

bool Data::desAcce(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp;
  buildVariable(tmp, byteStream);  // accx
  imugps_.a[0] = tmp * 16 * 9.8 / 32768;
  buildVariable(tmp, byteStream);  // accy
  imugps_.a[1] = tmp * 16 * 9.8 / 32768;
  buildVariable(tmp, byteStream);  // accz
  imugps_.a[2] = tmp * 16 * 9.8 / 32768;
  buildVariable(tmp, byteStream);
  imugps_.temperature = tmp / 100.0;
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.a[2] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desGyro(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp;
  buildVariable(tmp, byteStream);  // wx
  imugps_.w[0] = tmp * 2000.0 * 3.1415926 / (32768 * 180.0);
  buildVariable(tmp, byteStream);  // wy
  imugps_.w[1] = tmp * 2000.0 * 3.1415926 / (32768 * 180.0);
  buildVariable(tmp, byteStream);  // wz
  imugps_.w[2] = tmp * 2000.0 * 3.1415926 / (32768 * 180.0);
  buildVariable(tmp, byteStream);
  imugps_.temperature = tmp / 100.0;
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.w[0] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desLola(ecl::PushAndPop<unsigned char> &byteStream) {
  buildVariable(imugps_.longtitude, byteStream);
  buildVariable(imugps_.latitude, byteStream);
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.pressure << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desMag(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp;
  buildVariable(tmp, byteStream);  // magx
  imugps_.mag[0] = tmp;
  buildVariable(tmp, byteStream);  // magy
  imugps_.mag[1] = tmp;
  buildVariable(tmp, byteStream);  // magz
  imugps_.mag[2] = tmp;
  buildVariable(tmp, byteStream);
  imugps_.temperature = tmp / 100.0;
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.mag[0] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desPalt(ecl::PushAndPop<unsigned char> &byteStream) {
  buildVariable(imugps_.pressure, byteStream);
  buildVariable(imugps_.altitude, byteStream);
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.pressure << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desQuat(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp;
  buildVariable(tmp, byteStream);
  imugps_.q[0] = tmp / 32768.0;
  buildVariable(tmp, byteStream);
  imugps_.q[1] = tmp / 32768.0;
  buildVariable(tmp, byteStream);
  imugps_.q[2] = tmp / 32768.0;
  buildVariable(tmp, byteStream);
  imugps_.q[3] = tmp / 32768.0;
  if (byteStream.size() == 0) {
    std::cout << imugps_.q[0] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desSate(ecl::PushAndPop<unsigned char> &byteStream) {
  buildVariable(imugps_.satelites, byteStream);
  uint16_t tmp;
  buildVariable(tmp, byteStream);
  imugps_.gpsa[0] = tmp / 100.0;
  buildVariable(tmp, byteStream);
  imugps_.gpsa[1] = tmp / 100.0;
  buildVariable(tmp, byteStream);
  imugps_.gpsa[2] = tmp / 100.0;
  if (byteStream.size() == 0) {
    std::cout << imugps_.gpsa[0] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desState(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp;
  buildVariable(tmp, byteStream);  // D0
  imugps_.d[0] = tmp;
  buildVariable(tmp, byteStream);  // D1
  imugps_.d[1] = tmp;
  buildVariable(tmp, byteStream);  // D2
  imugps_.d[2] = tmp;
  buildVariable(tmp, byteStream);  // D3
  imugps_.d[3] = tmp;
  if (byteStream.size() == 0) {
    std::cout << imugps_.d[0] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desRpy(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp;
  buildVariable(tmp, byteStream);  // roll
  imugps_.rpy[0] = tmp * 3.1415926 / 32768;
  buildVariable(tmp, byteStream);  // pitch
  imugps_.rpy[1] = tmp * 3.1415926 / 32768;
  buildVariable(tmp, byteStream);  // yaw
  imugps_.rpy[2] = tmp * 3.1415926 / 32768;
  buildVariable(tmp, byteStream);
  imugps_.temperature = tmp / 100.0;
  if (byteStream.size() == 0) {
    std::cout << imugps_.rpy[0] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desGpsv(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp;
  buildVariable(tmp, byteStream);  // gpsh
  imugps_.gpsh = tmp / 10.0;
  buildVariable(tmp, byteStream);  // gpsy
  imugps_.gpsy = tmp / 10.0;
  buildVariable(imugps_.gpsv, byteStream);  // accz
  imugps_.gpsv = imugps_.gpsv / 1000.0;
  if (byteStream.size() == 0) {
    std::cout << imugps_.gpsv << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::deserialise(ecl::PushAndPop<unsigned char> &byteStream) {
  uint8_t flag;
  buildVariable(flag, byteStream);
  switch (flag) {
    case Flags::TIME:
      desTime(byteStream);
      break;
    case Flags::ACCE:
      desAcce(byteStream);
      break;
    case Flags::GYRO:
      desGyro(byteStream);
      break;
    case Flags::RPY:
      desRpy(byteStream);
      break;
    case Flags::MAG:
      desMag(byteStream);
      break;
    case Flags::STATE:
      desState(byteStream);
      break;
    case Flags::PALT:
      desPalt(byteStream);
      break;
    case Flags::LOLA:
      desLola(byteStream);
      break;
    case Flags::GPSV:
      desGpsv(byteStream);
      break;
    case Flags::QUAT:
      desQuat(byteStream);
      break;
    case Flags::SATE:
      desSate(byteStream);
      break;
    default:
      break;
  }

  return true;
}

void Data::build_special_variable(float &variable,
                                  ecl::PushAndPop<unsigned char> &byteStream) {
  if (byteStream.size() < 2) return;

  unsigned char a, b;
  buildVariable(a, byteStream);
  buildVariable(b, byteStream);
  variable = ((unsigned int)(a & 0x0f)) / 100.0;

  variable += ((unsigned int)(a >> 4)) / 10.0;

  variable += (unsigned int)(b & 0x0f);

  variable += ((unsigned int)(b >> 4)) * 10;
}

}  // namespace xbot
