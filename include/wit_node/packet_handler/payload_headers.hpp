/**
 * @file include/xbot_arm_driver/packet_handler/payload_headers.hpp
 *
 * @brief Byte id's for the individual payload headers.
 *
 * Each part of a xbot packet carries one or more payload chunks. Each chunk
 * is id'd by one of the values here.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_arm_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef XBOT_PAYLOAD_HEADERS_HPP_
#define XBOT_PAYLOAD_HEADERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace wit {

class Header {
 public:
  enum PayloadType {
    TIME = 0x50,
    ACCE = 0x51,
    GYRO = 0x52,
    YPR = 0x53,
    MAG = 0x54,
    STATE = 0x55,
    PALT = 0x56,
    LOLA = 0x57,
    GPSV = 0x58,
    QUAT = 0x59,
    SATE = 0x5a
  };
};

}  // namespace xbot

#endif /* XBOT_PAYLOAD_HEADERS_HPP_ */
