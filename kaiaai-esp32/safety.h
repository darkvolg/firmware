// Safety layer added in the pet fork (not part of upstream KAIA.AI firmware)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Three protections, all configured in config.yaml under "safety:"
//   1. /cmd_vel watchdog: no command for cmd_vel.timeout -> stop
//   2. stall: motor driven, wheel not turning for stall.timeout -> stop,
//      commands ignored for stall.hold
//   3. battery, in steps: low -> report only; stop -> motors and LiDAR off;
//      sleep -> deep sleep, wake every battery.wake seconds to re-check
// Stall and battery events go to /rosout as "SAFETY: ..." warnings.

#pragma once

#include <WiFi.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include "robot_config.h"
#include "util.h"
#include "motors.h"
#include "lidar.h"
#include "ros.h"
#include "adc.h"

extern CONFIG cfg;
extern float ramp_target_rpm_right;
extern float ramp_target_rpm_left;
extern float odom_target_d_dist;
extern float odom_target_d_yaw;

enum safety_bat_t {
  SAFETY_BAT_ABSENT = -1, // no battery sensed (USB power on the bench), no limits
  SAFETY_BAT_OK = 0,
  SAFETY_BAT_LOW,         // still drives, time to ask for a charge
  SAFETY_BAT_STOP,        // motors and LiDAR off
  SAFETY_BAT_SLEEP,       // deep sleep
};

const unsigned long SAFETY_BAT_SAMPLE_PERIOD_US = 100*1000;
const float SAFETY_BAT_FILTER = 0.2f; // motor load makes the voltage jumpy
const uint8_t SAFETY_BAT_BOOT_SAMPLES = 16;

RTC_DATA_ATTR bool safety_bat_sleeping = false; // survives deep sleep
safety_bat_t safety_bat_state = SAFETY_BAT_ABSENT;
safety_bat_t safety_bat_candidate = SAFETY_BAT_ABSENT;
unsigned long safety_bat_candidate_time_us = 0;
unsigned long safety_bat_sample_time_us = 0;
float safety_bat_volts = -1;

unsigned long safety_cmd_vel_time_us = 0;
bool safety_stall_hold = false;
unsigned long safety_stall_time_us = 0;

// Detects a motor that is driven but does not turn in the commanded
// direction. A reversed encoder looks the same and is caught too.
class StallMonitor {
  public:
    void reset() {
      dir_prev = 0;
    }

    bool check(MotorController & motor, unsigned long now_us) {
      float target_rpm = motor.getTargetRPM();
      int8_t dir = target_rpm > 0 ? 1 : (target_rpm < 0 ? -1 : 0);
      bool driven = dir != 0 &&
        abs(motor.getCurrentPWM()) >= cfg.safety_stall_pwm_min;
      long enc = motor.getEncoderValue();

      if (!driven) {
        dir_prev = 0;
        return false;
      }

      if (dir != dir_prev) {
        dir_prev = dir;
        enc_ref = enc;
        start_us = now_us;
        return false;
      }

      if (now_us - start_us < (unsigned long)(cfg.safety_stall_timeout * 1e6))
        return false;

      long moved = (enc - enc_ref) * dir;
      long moved_min = lround(cfg.safety_stall_rpm_min / 60.0f *
        cfg.safety_stall_timeout * motor.getEncoderTPR());
      enc_ref = enc;
      start_us = now_us;
      return moved < moved_min;
    }

  protected:
    int8_t dir_prev = 0;
    long enc_ref = 0;
    unsigned long start_us = 0;
};

StallMonitor stallLeft, stallRight;

inline bool safetyBatEnabled() {
  return cfg.safety_bat_low > 0 || cfg.safety_bat_stop > 0 ||
    cfg.safety_bat_sleep > 0;
}

inline bool safetyMotionBlocked() {
  return safety_stall_hold || safety_bat_state >= SAFETY_BAT_STOP;
}

inline bool safetyLidarBlocked() {
  return safety_bat_state >= SAFETY_BAT_STOP;
}

inline void safetyOnCmdVel() {
  safety_cmd_vel_time_us = esp_timer_get_time();
}

// Zero all motion targets and brake right away, without waiting for the PID
void safetyStopMotors() {
  ramp_target_rpm_right = 0;
  ramp_target_rpm_left = 0;
  odom_target_d_dist = 0;
  odom_target_d_yaw = 0;
  setMotorSpeeds(0, 0);
  brakeMotor(false);
  brakeMotor(true);
}

// Below this the battery is considered not connected
float safetyBatAbsentVolts() {
  float lowest = cfg.safety_bat_sleep > 0 ? cfg.safety_bat_sleep :
    (cfg.safety_bat_stop > 0 ? cfg.safety_bat_stop : cfg.safety_bat_low);
  return lowest * 0.5f;
}

safety_bat_t safetyBatLevel(float volts, float margin) {
  if (cfg.safety_bat_sleep > 0 && volts <= cfg.safety_bat_sleep + margin)
    return SAFETY_BAT_SLEEP;
  if (cfg.safety_bat_stop > 0 && volts <= cfg.safety_bat_stop + margin)
    return SAFETY_BAT_STOP;
  if (cfg.safety_bat_low > 0 && volts <= cfg.safety_bat_low + margin)
    return SAFETY_BAT_LOW;
  return SAFETY_BAT_OK;
}

// Steps down at a threshold, back up only at threshold + hysteresis
safety_bat_t safetyBatNext(float volts) {
  if (volts < safetyBatAbsentVolts())
    return SAFETY_BAT_ABSENT;

  safety_bat_t down = safetyBatLevel(volts, 0);
  if (safety_bat_state == SAFETY_BAT_ABSENT || down > safety_bat_state)
    return down;

  safety_bat_t up = safetyBatLevel(volts, cfg.safety_bat_hysteresis);
  return up < safety_bat_state ? up : safety_bat_state;
}

const char* safetyBatName(safety_bat_t state) {
  switch (state) {
    case SAFETY_BAT_OK: return "ok";
    case SAFETY_BAT_LOW: return "low";
    case SAFETY_BAT_STOP: return "stop";
    case SAFETY_BAT_SLEEP: return "sleep";
    default: return "absent";
  }
}

void safetyReleaseMotorPins() {
  const uint8_t pins[] = {
    cfg.mot_left_drv_gpio_in1_pwm, cfg.mot_left_drv_gpio_in2_cw,
    cfg.mot_right_drv_gpio_in1_pwm, cfg.mot_right_drv_gpio_in2_cw};

  gpio_deep_sleep_hold_dis();
  for (uint8_t i = 0; i < 4; i++)
    if (pins[i] != cfg.UNDEFINED_GPIO)
      gpio_hold_dis((gpio_num_t) pins[i]);
}

// Does not return. The motor driver inputs are held in the stop state
// through deep sleep, otherwise they float.
void safetyBatterySleep() {
  const uint8_t pins[] = {
    cfg.mot_left_drv_gpio_in1_pwm, cfg.mot_left_drv_gpio_in2_cw,
    cfg.mot_right_drv_gpio_in1_pwm, cfg.mot_right_drv_gpio_in2_cw};
  bool pwm_cw = cfg.motor_driver_type == "PWM_CW";

  if (lidar)
    lidar->stop();

  for (uint8_t i = 0; i < 4; i++) {
    if (pins[i] == cfg.UNDEFINED_GPIO)
      continue;

    gpio_hold_dis((gpio_num_t) pins[i]);
    #if ESP_IDF_VERSION_MAJOR >= 5
    ledcDetach(pins[i]);
    #else
    ledcDetachPin(pins[i]);
    #endif
    pinMode(pins[i], OUTPUT);
    // PWM_CW stops with the PWM pin HIGH, IN1_IN2 with both inputs LOW
    bool is_pwm_pin = (i % 2) == 0;
    digitalWrite(pins[i], (pwm_cw && is_pwm_pin) ? HIGH : LOW);
    gpio_hold_en((gpio_num_t) pins[i]);
  }
  gpio_deep_sleep_hold_en();

  safety_bat_sleeping = true;
  if (cfg.safety_bat_wake > 0)
    esp_sleep_enable_timer_wakeup((uint64_t)(cfg.safety_bat_wake * 1e6));

  Serial.println("SAFETY: entering deep sleep");
  Serial.flush();
  esp_deep_sleep_start();
}

void safetyBatSet(safety_bat_t next) {
  safety_bat_t prev = safety_bat_state;
  safety_bat_state = next;

  // Act first, logging to ROS may block
  if (next >= SAFETY_BAT_STOP) {
    safetyStopMotors();
    lidar->stop();
  } else if (prev >= SAFETY_BAT_STOP && WiFi.status() == WL_CONNECTED)
    lidar->start();

  String s = "SAFETY: battery ";
  s = s + safetyBatName(next) + " " + String(safety_bat_volts) + "V";
  logMsg((char*)s.c_str(), rcl_interfaces__msg__Log__WARN);

  if (next == SAFETY_BAT_SLEEP)
    safetyBatterySleep();
}

void safetyBatteryLoop(unsigned long now_us) {
  if (!safetyBatEnabled())
    return;

  if (now_us - safety_bat_sample_time_us < SAFETY_BAT_SAMPLE_PERIOD_US)
    return;
  safety_bat_sample_time_us = now_us;

  float volts = getBatteryMilliVolts() * 0.001f;
  safety_bat_volts = safety_bat_volts < 0 ? volts :
    safety_bat_volts + SAFETY_BAT_FILTER * (volts - safety_bat_volts);

  safety_bat_t next = safetyBatNext(safety_bat_volts);
  if (next == safety_bat_state) {
    safety_bat_candidate = next;
    return;
  }

  // A new level has to last safety.battery.delay to be accepted
  if (next != safety_bat_candidate) {
    safety_bat_candidate = next;
    safety_bat_candidate_time_us = now_us;
    return;
  }

  if (now_us - safety_bat_candidate_time_us <
    (unsigned long)(cfg.safety_bat_delay * 1e6))
    return;

  safetyBatSet(next);
}

// Call from setup() after setupADC() and before setupMotors()
void safetySetup() {
  bool was_sleeping = safety_bat_sleeping;
  safety_bat_sleeping = false;

  Serial.print("Safety: cmd_vel timeout ");
  Serial.print(cfg.safety_cmd_vel_timeout);
  Serial.print("s, stall timeout ");
  Serial.print(cfg.safety_stall_timeout);
  Serial.print("s, battery low/stop/sleep ");
  Serial.print(cfg.safety_bat_low);
  Serial.print('/');
  Serial.print(cfg.safety_bat_stop);
  Serial.print('/');
  Serial.print(cfg.safety_bat_sleep);
  Serial.println("V");

  if (safetyBatEnabled()) {
    float mv = 0;
    for (uint8_t i = 0; i < SAFETY_BAT_BOOT_SAMPLES; i++) {
      mv += getBatteryMilliVolts();
      delay(2);
    }
    safety_bat_volts = mv * 0.001f / SAFETY_BAT_BOOT_SAMPLES;

    if (safety_bat_volts < safetyBatAbsentVolts()) {
      Serial.println("Safety: battery NOT sensed, battery protection inactive");
    } else {
      // Woken up to re-check: stay asleep until the battery is charged
      float sleep_volts = cfg.safety_bat_sleep +
        (was_sleeping ? cfg.safety_bat_hysteresis : 0);
      if (cfg.safety_bat_sleep > 0 && safety_bat_volts <= sleep_volts) {
        Serial.print("SAFETY: battery ");
        Serial.print(safety_bat_volts);
        Serial.println("V too low to run");
        safetyBatterySleep();
      }

      safety_bat_state = safetyBatLevel(safety_bat_volts, 0);
      safety_bat_candidate = safety_bat_state;
      Serial.print("Safety: battery ");
      Serial.print(safetyBatName(safety_bat_state));
      Serial.print(" ");
      Serial.print(safety_bat_volts);
      Serial.println("V");
    }
  }

  safetyReleaseMotorPins();
}

// Call from loop() after the micro-ROS executor has run
void safetyLoop() {
  unsigned long now_us = esp_timer_get_time();

  bool moving = ramp_target_rpm_left != 0 || ramp_target_rpm_right != 0 ||
    motorLeft.getTargetRPM() != 0 || motorRight.getTargetRPM() != 0;

  if (cfg.safety_cmd_vel_timeout > 0 && moving &&
    now_us - safety_cmd_vel_time_us >
    (unsigned long)(cfg.safety_cmd_vel_timeout * 1e6)) {
    safetyStopMotors();
    // Serial only: the PC is likely gone, logging to ROS would block
    printlnNB("SAFETY: cmd_vel timeout, motors stopped");
  }

  if (safety_stall_hold) {
    if (now_us - safety_stall_time_us >=
      (unsigned long)(cfg.safety_stall_hold * 1e6))
      safety_stall_hold = false;
  } else if (cfg.safety_stall_timeout > 0) {
    bool left = stallLeft.check(motorLeft, now_us);
    bool right = stallRight.check(motorRight, now_us);

    if (left || right) {
      safetyStopMotors();
      safety_stall_hold = true;
      safety_stall_time_us = now_us;
      stallLeft.reset();
      stallRight.reset();

      String s = "SAFETY: stall ";
      s = s + (left && right ? "both" : (left ? "left" : "right"));
      logMsg((char*)s.c_str(), rcl_interfaces__msg__Log__WARN);
    }
  }

  safetyBatteryLoop(now_us);
}
