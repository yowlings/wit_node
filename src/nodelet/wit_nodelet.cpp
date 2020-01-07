#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>
#include "../../include/wit_node/wit_ros.hpp"

namespace wit {

class WitNodelet : public nodelet::Nodelet {
 public:
  WitNodelet() : shutdown_requested_(false){};
  ~WitNodelet() {
    NODELET_DEBUG_STREAM("Wit : waiting for update thread to finish.");
    shutdown_requested_ = true;
    update_thread_.join();
  }

  virtual void onInit() {
    NODELET_DEBUG_STREAM("Wit: initialising nodelet...");
    string nodelet_name = this->getName();
    wit_.reset(new WitRos(nodelet_name));
    if (wit_->init(this->getPrivateNodeHandle())) {
      update_thread_.start(&WitNodelet::update, *this);
      NODELET_INFO_STREAM("Wit : initialised.");

    } else {
      NODELET_ERROR_STREAM("Wit : could not initialise! Please restart.");
    }
  }

 private:
  void update() {
    Rate spin_rate(10);
    while (!shutdown_requested_ && ok()) {
      wit_->update();
      spin_rate.sleep();
    }
  }

  boost::shared_ptr<WitRos> wit_;
  ecl::Thread update_thread_;
  bool shutdown_requested_;
};
}
PLUGINLIB_EXPORT_CLASS(wit::WitNodelet, nodelet::Nodelet);
