// Minimal PC stand-ins for the Arduino/ESP32/micro-ROS pieces safety.h uses,
// so its logic can be tested without the robot. See ../run_test.ps1

#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#define RTC_DATA_ATTR
#define ESP_IDF_VERSION_MAJOR 4
#define OUTPUT 1
#define HIGH 1
#define LOW 0
#define WL_CONNECTED 3

const uint8_t rcl_interfaces__msg__Log__WARN = 30;

class String {
  public:
    String() {}
    String(const char * str) : s(str) {}
    String(float value) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%.2f", value);
      s = buf;
    }
    String operator+(const String & other) const {
      String r;
      r.s = s + other.s;
      return r;
    }
    String operator+(const char * other) const {
      String r;
      r.s = s + other;
      return r;
    }
    bool operator==(const char * other) const { return s == other; }
    const char * c_str() const { return s.c_str(); }
    std::string s;
};

struct SerialStub {
  template<class T> void print(T) {}
  template<class T> void println(T) {}
  void println() {}
  void flush() {}
} Serial;

std::vector<std::string> serial_nb_log;
void printlnNB(const String & s) { serial_nb_log.push_back(s.s); }

struct WiFiStub {
  int status() { return WL_CONNECTED; }
} WiFi;

struct CONFIG {
  static const uint8_t UNDEFINED_GPIO = 255;
  String motor_driver_type = "IN1_IN2";
  uint8_t mot_left_drv_gpio_in1_pwm = 26;
  uint8_t mot_left_drv_gpio_in2_cw = 25;
  uint8_t mot_right_drv_gpio_in1_pwm = 23;
  uint8_t mot_right_drv_gpio_in2_cw = 22;
  float safety_cmd_vel_timeout = 0.5f;
  float safety_stall_timeout = 1.0f;
  float safety_stall_rpm_min = 3.0f;
  float safety_stall_pwm_min = 0.15f;
  float safety_stall_hold = 2.0f;
  float safety_bat_low = 14.0f;
  float safety_bat_stop = 13.2f;
  float safety_bat_sleep = 12.8f;
  float safety_bat_hysteresis = 0.5f;
  float safety_bat_delay = 3.0f;
  float safety_bat_wake = 60.0f;
};
CONFIG cfg;

class MotorController {
  public:
    float getTargetRPM() { return target; }
    float getCurrentPWM() { return pwm; }
    long getEncoderValue() { return lround(enc); }
    float getEncoderTPR() { return tpr; }
    float target = 0;
    float pwm = 0;
    double enc = 0;
    float tpr = 1496; // JGA25-370B 1:34, 11 PPR, x4
};
MotorController motorLeft, motorRight;

int brake_calls = 0;
void brakeMotor(bool) { brake_calls++; }
void setMotorSpeeds(float rpm_left, float rpm_right) {
  motorLeft.target = rpm_left;
  motorRight.target = rpm_right;
}

struct LDS {
  bool running = true;
  void start() { running = true; }
  void stop() { running = false; }
};
LDS lidar_stub;
LDS * lidar = &lidar_stub;

std::vector<std::string> ros_log;
void logMsg(char * msg, uint8_t) { ros_log.push_back(msg); }

float fake_battery_mv = 0;
float getBatteryMilliVolts() { return fake_battery_mv; }

uint64_t fake_time_us = 0;
int64_t esp_timer_get_time() { return (int64_t) fake_time_us; }
void delay(unsigned long) {}

typedef int gpio_num_t;
void gpio_hold_en(gpio_num_t) {}
void gpio_hold_dis(gpio_num_t) {}
void gpio_deep_sleep_hold_en() {}
void gpio_deep_sleep_hold_dis() {}
void ledcDetachPin(uint8_t) {}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void esp_sleep_enable_timer_wakeup(uint64_t) {}

// The real call never returns
struct DeepSleepEntered {};
void esp_deep_sleep_start() { throw DeepSleepEntered(); }
