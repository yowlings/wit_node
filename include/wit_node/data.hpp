
#ifndef DATA_HPP__
#define DATA_HPP__

#include "packet_handler/payload_base.hpp"

#include <stdint.h>

namespace wit {

/*****************************************************************************
** Interface
*****************************************************************************/

class Data : public packet_handler::payloadBase {
 public:
  Data() : packet_handler::payloadBase(false, 30){};

  struct IMUGPS {
    double timestamp;
    double a[3];
    double temperature;
    double w[3];
    double rpy[3];
    double mag[3];
    uint16_t d[4];
    float pressure;
    float altitude;
    float longtitude;
    float latitude;
    float gpsv;
    double gpsh;
    double gpsy;
    double q[4];
    uint16_t satelites;
    double gpsa[3];

  } imugps_;

  struct Flags {
    static const uint8_t TIME = 0x50;
    static const uint8_t ACCE = 0x51;
    static const uint8_t GYRO = 0x52;
    static const uint8_t RPY = 0x53;
    static const uint8_t MAG = 0x54;
    static const uint8_t STATE = 0x55;
    static const uint8_t PALT = 0x56;
    static const uint8_t LOLA = 0x57;
    static const uint8_t GPSV = 0x58;
    static const uint8_t QUAT = 0x59;
    static const uint8_t SATE = 0x5a;
  };

  bool serialise(ecl::PushAndPop<unsigned char> &byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> &byteStream);
  bool desTime(ecl::PushAndPop<unsigned char> &byteStream);
  bool desAcce(ecl::PushAndPop<unsigned char> &byteStream);
  bool desGyro(ecl::PushAndPop<unsigned char> &byteStream);
  bool desRpy(ecl::PushAndPop<unsigned char> &byteStream);
  bool desMag(ecl::PushAndPop<unsigned char> &byteStream);
  bool desState(ecl::PushAndPop<unsigned char> &byteStream);
  bool desPalt(ecl::PushAndPop<unsigned char> &byteStream);
  bool desLola(ecl::PushAndPop<unsigned char> &byteStream);
  bool desGpsv(ecl::PushAndPop<unsigned char> &byteStream);
  bool desQuat(ecl::PushAndPop<unsigned char> &byteStream);
  bool desSate(ecl::PushAndPop<unsigned char> &byteStream);

  void build_special_variable(float &variable,
                              ecl::PushAndPop<unsigned char> &byteStream);
};
}

#endif /* DATA_HPP__ */
