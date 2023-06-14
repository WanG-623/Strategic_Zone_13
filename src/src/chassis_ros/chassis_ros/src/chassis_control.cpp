#include "chassis_control.h"
#include "can_communication.h"

//SerialDriver *chassis_can = nullptr;
CHASSIS_CAN *chassis_can = nullptr;

bool chassisInit(std::string &dev)
{
  //chassis_can = new SerialDriver();
  //return chassis_can->init(PORT, dev, 115200);
  chassis_can = new CHASSIS_CAN();
  return chassis_can->init("can2",1000000);
}

void exit()
{
  chassis_can->close();
  exit(-1);
}

bool getBatteryInfo(float &vol, float &cur, int &percent)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->getEntry(INDEX_SYS_VOLTAGE, &vol) &&
         chassis_can->getEntry(INDEX_SYS_CURRENT, &cur) &&
         chassis_can->getEntry(INDEX_SYS_POWER_PERCENTAGE, &percent);
}

bool getChassisCollision(int &collision)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->getEntry(INDEX_SYS_COLLISION, &collision);
}

bool getChassisState(SystemState &state)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->getEntry(INDEX_SYS_STATE, (int *)&state);
}

bool getVersion(char *text)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->getEntry(INDEX_SYS_VERSION, text);
}

bool setText(const char *text)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->setEntry(INDEX_SYS_TEXT, text);
}

bool setCharge(int charge)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->setEntry(INDEX_SYS_CHARGE, &charge);
}

bool setAlarm(int alarm)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->setEntry(INDEX_SYS_ALARM, &alarm);
}

bool setLight(int light)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->setEntry(INDEX_SYS_LIGHT, &light);
}

bool setStateLight(int light)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->setEntry(INDEX_STATE_LIGHT, &light);
}

/**
 * @brief 控制车的x方向的线速度和z方向的转动速度
 * @param aim_x_vel  m/s
 * @param aim_z_omega rad/s
 */
bool chassisControl(float aim_x_vel, float aim_z_omega)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->setEntry(INDEX_AIM_CHASSIS_VEL, &aim_x_vel) &&
         chassis_can->setEntry(INDEX_AIM_CHASSIS_POS_OR_OMEGA, &aim_z_omega);
}
/**
 * @brief 更新x线速度和z转速度
 * @param x_vel m/s
 * @param z_omega rad/s
 */
bool getChassisInfo(float &x_vel, float &z_omega)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->getEntry(INDEX_CHASSIS_VEL, &x_vel) &&
         chassis_can->getEntry(INDEX_CHASSIS_POS_OR_OMEGA, &z_omega);
}
/**
 * @brief 更新里程计的信息
 * @param odom_x
 * @param odom_y
 * @param odom_theta 转角
 */
bool getChassisOdom(float &odom_x, float &odom_y, float &odom_theta)
{
  ASSERT(chassis_can != nullptr, exit());
  return chassis_can->getEntry(INDEX_CHASSIS_ODOM_X, &odom_x) &&
         chassis_can->getEntry(INDEX_CHASSIS_ODOM_Y, &odom_y) &&
         chassis_can->getEntry(INDEX_CHASSIS_ODOM_THETA, &odom_theta);
}
