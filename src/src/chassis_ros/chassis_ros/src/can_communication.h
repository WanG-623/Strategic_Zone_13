
#include <string>
//#include "chassis_can.hpp"
#include <socket_can/socket_can.hpp>

#include <cstdint>
#include <string>
#include <iostream>
//#define ASSERT(expr, fail) (static_cast<bool>(expr) ? void(0) : (fail))


//namespace chassis_can {

class CHASSIS_CAN {
 public:
  CHASSIS_CAN();

  CHASSIS_CAN(std::string port);

  ~CHASSIS_CAN();
  bool init(const char* device, int bitRate);
  void close();

  bool setEntry(int index, const float* data_address);
  bool setEntry(int index, const int* data_address);
  bool setEntry(int index, const char* data_address);

  bool getEntry(int index, float* data_address);
  bool getEntry(int index, int* data_address);
  bool getEntry(int index, char* data_address);

 private:
  bool handleTimeout(int);

  int fd_;
  bool isOpened_;
  int conn_timeout_cnt_;
  socket_can::SocketCAN can_;
  struct can_msg_
  {
    uint8_t a;
    uint8_t b;
    uint8_t c;
    uint8_t d;
  };
  
  union can_msg
  {
    can_msg_ m;
    float f;
    /* data */
  };
  union can_msg_int
  {
    can_msg_ m;
    int i;
    /* data */
  };
  

};
//}