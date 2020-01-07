
#include "../../include/wit_node/wit_driver.hpp"
#include <unistd.h>
#include <cmath>
#include <ecl/converters.hpp>
#include <ecl/geometry/angle.hpp>
#include <ecl/math.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/time/sleep.hpp>
#include <ecl/time/timestamp.hpp>
#include <fstream>
#include <stdexcept>

namespace wit {

bool PacketFinder::checkSum() {
  unsigned int packet_size(buffer.size());
  unsigned char cs(0);
  for (unsigned int i = 0; i < packet_size - 2; i++) {
    cs += buffer[i];
  }

  return (cs == buffer[packet_size - 1]) ? false : true;
}

WitDriver::WitDriver()
    : shutdown_requested_(false),
      connected_(false),
      alive_(false),
      yaw_offset_(0.0 / 0.0) {}

/**
 * Shutdown the driver - make sure we wait for the thread to finish.
 */
WitDriver::~WitDriver() {
  shutdown_requested_ = true;  // thread's spin() will catch this and terminate
  thread_spin_.join();
}

void WitDriver::init(Parameter &param) {
  this->param_ = param;
  std::string ns = param_.ns;

  // connect signals
  sig_stream_data.connect(ns + std::string("/stream_data"));

  try {
    serial.open(param_.port_, ecl::BaudRate_115200, ecl::DataBits_8,
                ecl::StopBits_1, ecl::NoParity);
    serial.block(4000);  // blocks by default, but just to be clear!
    connected_ = true;

  } catch (const ecl::StandardException &e) {
    connected_ = false;
    if (e.flag() == ecl::NotFoundError) {
    } else {
      throw ecl::StandardException(LOC, e);
    }
  }

  ecl::PushAndPop<unsigned char> stx(1, 0);
  ecl::PushAndPop<unsigned char> etx(1);
  stx.push_back(0x55);
  packet_finder.configure(ns, stx, etx, 1, 256, 1, true);
  thread_spin_.start(&WitDriver::spin, *this);
}

void WitDriver::lockDataAccess() { data_mutex_.lock(); }

/**
 * Unlock a previously locked data access privilege.
 * @sa lockDataAccess()
 */
void WitDriver::unlockDataAccess() { data_mutex_.unlock(); }

/**
 * @brief Performs a scan looking for incoming data packets.
 *
 * Sits on the device waiting for incoming and then parses it, and signals
 * that an update has occured.
 *
 * Or, if in simulation, just loopsback the motor devices.
 */

void WitDriver::spin() {
  ecl::TimeStamp last_signal_time;
  ecl::Duration timeout(0.1);
  unsigned char buf[256];

  while (!shutdown_requested_) {
    /*********************
     ** Checking Connection
     **********************/
    if (!serial.open()) {
      try {
        serial.open(param_.port_, ecl::BaudRate_115200, ecl::DataBits_8,
                    ecl::StopBits_1, ecl::NoParity);
        serial.block(4000);  // blocks by default, but just to be clear!
        connected_ = true;
      } catch (const ecl::StandardException &e) {
        connected_ = false;
        alive_ = false;
        // windows throws OpenError if not connected
        if (e.flag() == ecl::NotFoundError) {
          std::cout
              << "device does not (yet) available on this port, waiting..."
              << std::endl;
        } else if (e.flag() == ecl::OpenError) {
          std::cout << "device failed to open, waiting... ["
                    << std::string(e.what()) << "]" << std::endl;
          ;
        } else {
          throw ecl::StandardException(LOC, e);
        }
        ecl::Sleep(5)();  // five seconds
        continue;
      }
    }

    /*********************
     ** Read Incoming
     **********************/
    int n = serial.read((char *)buf, 1);
    //    std::cout << int(buf[0]) << std::endl;
    if (n == 0) {
      continue;
    }
    bool find_packet = packet_finder.update(buf, n);

    if (find_packet) {
      PacketFinder::BufferType local_buffer;
      packet_finder.getBuffer(local_buffer);
      packet_finder.getPayload(data_buffer);
      lockDataAccess();
      unsigned short flag = data_buffer[0];
      while (data_buffer.size() > 0) {
        data_.deserialise(data_buffer);
        if (flag == Data::Flags::RPY) {
          if (isnan(yaw_offset_)) yaw_offset_ = data_.imugps_.rpy[2];
          relate_yaw_ = data_.imugps_.rpy[2] - yaw_offset_;
        }
      }
      unlockDataAccess();
      alive_ = true;
      if (flag == Data::Flags::ACCE) {
        sig_stream_data.emit();
      }
      last_signal_time.stamp();

    } else {
      if (alive_ && ((ecl::TimeStamp() - last_signal_time) > timeout)) {
        alive_ = false;
      }
    }
  }
}

}  // namespace WitDriver
