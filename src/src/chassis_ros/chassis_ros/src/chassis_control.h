#ifndef __CHASSIS_CONTROL_H_
#define __CHASSIS_CONTROL_H_

#define ASSERT(expr, fail) (static_cast<bool>(expr) ? void(0) : (fail))
#include <string>

#include "od_index.h"

bool chassisInit(std::string &dev);
bool chassisControl(float aim_x_vel, float aim_z_omega);
bool getChassisInfo(float &x_vel, float &z_omega);
bool getChassisOdom(float &odom_x, float &odom_y, float &odom_theta);

bool getBatteryInfo(float &vol, float &cur, int &percent);
bool getChassisState(SystemState &state);
bool getChassisCollision(int &collision);
bool getVersion(char *text);

// 50个汉字(GB2312编码)或者100个字符,
bool setText(const char *text);

// 0 / 1
bool setCharge(int charge);

// 0 / 1
bool setAlarm(int alarm);

// 0 ~ 100
bool setLight(int light);

bool setStateLight(int light);

#endif
