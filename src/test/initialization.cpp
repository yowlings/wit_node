
#include <unistd.h>
#include <ecl/time.hpp>
#include <iostream>
#include "../../include/wit_node/wit_driver.hpp"
using namespace wit;
class WitManager {
 public:
  WitManager() {
    wit::Parameter param;
    param.port_ = "/dev/ttyUSB0";
    param.ns = "wit";
    param.baut_rate_ = 115200;
    try {
      wd_.init(param);
    } catch (ecl::StandardException &e) {
      std::cout << e.what();
    }
  }

 private:
  WitDriver wd_;
};

int main() {
  WitManager wm;
  ecl::Sleep()(500);
  return 0;
}
