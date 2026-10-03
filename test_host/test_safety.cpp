// PC test of the logic in kaiaai-esp32/safety.h. Run: run_test.ps1

#include "safety.h"

float ramp_target_rpm_right = 0;
float ramp_target_rpm_left = 0;
float odom_target_d_dist = 0;
float odom_target_d_yaw = 0;

int failures = 0;
#define CHECK(cond) do { if (!(cond)) { failures++; \
  printf("  FAIL line %d: %s\n", __LINE__, #cond); } } while (0)

const unsigned STEP_MS = 10;
bool wheels_follow = true;   // encoders follow the target RPM
bool encoder_reversed = false;
bool pc_alive = true;        // /cmd_vel keeps coming
float cmd_rpm = 0;

void reset() {
  // Close to the 32-bit microsecond counter wrap on purpose
  fake_time_us = 0xFFFFFFFFull - 3*1000*1000;
  cfg = CONFIG();
  motorLeft = MotorController();
  motorRight = MotorController();
  ramp_target_rpm_left = ramp_target_rpm_right = 0;
  odom_target_d_dist = odom_target_d_yaw = 0;
  brake_calls = 0;
  ros_log.clear();
  serial_nb_log.clear();
  lidar_stub.running = true;
  fake_battery_mv = 16000;
  wheels_follow = true;
  encoder_reversed = false;
  pc_alive = true;
  cmd_rpm = 0;

  safety_bat_sleeping = false;
  safety_bat_state = SAFETY_BAT_ABSENT;
  safety_bat_candidate = SAFETY_BAT_ABSENT;
  safety_bat_candidate_time_us = 0;
  safety_bat_sample_time_us = 0;
  safety_bat_volts = -1;
  safety_cmd_vel_time_us = 0;
  safety_stall_hold = false;
  safety_stall_time_us = 0;
  stallLeft.reset();
  stallRight.reset();
}

// What twist_sub_callback() and the PID do in the firmware, simplified
void step() {
  fake_time_us += STEP_MS * 1000;

  if (pc_alive) {
    safetyOnCmdVel();
    if (!safetyMotionBlocked()) {
      ramp_target_rpm_left = cmd_rpm;
      ramp_target_rpm_right = cmd_rpm;
    }
  }

  safetyLoop();

  if (safetyMotionBlocked())
    setMotorSpeeds(0, 0);
  else
    setMotorSpeeds(ramp_target_rpm_left, ramp_target_rpm_right);

  MotorController * motors[] = {&motorLeft, &motorRight};
  for (MotorController * m : motors) {
    m->pwm = m->target == 0 ? 0 : (m->target > 0 ? 0.5f : -0.5f);
    if (wheels_follow) {
      double ticks = m->target / 60.0 * m->tpr * STEP_MS * 0.001;
      m->enc += encoder_reversed ? -ticks : ticks;
    }
  }
}

void run(float seconds) {
  for (unsigned i = 0; i < (unsigned)(seconds * 1000 / STEP_MS); i++)
    step();
}

bool moving() {
  return motorLeft.target != 0 || motorRight.target != 0;
}

int count(const char * text) {
  int n = 0;
  for (auto & s : ros_log)
    if (s.find(text) != std::string::npos)
      n++;
  return n;
}

bool logged(const char * text) {
  return count(text) > 0;
}

void testCmdVelWatchdog() {
  printf("cmd_vel watchdog\n");
  reset();
  cmd_rpm = 100;
  run(5);
  CHECK(moving());
  CHECK(brake_calls == 0);

  pc_alive = false; // PC hangs, WiFi is fine
  run(0.4);
  CHECK(moving());
  run(0.2);
  CHECK(!moving());
  CHECK(brake_calls == 2);
  CHECK(serial_nb_log.size() == 1);

  run(5); // stays stopped, no repeated action
  CHECK(!moving());
  CHECK(brake_calls == 2);

  pc_alive = true; // PC is back
  run(0.1);
  CHECK(moving());

  reset();
  cfg.safety_cmd_vel_timeout = 0; // disabled
  cmd_rpm = 100;
  run(1);
  pc_alive = false;
  run(5);
  CHECK(moving());
}

void testStall() {
  printf("stall\n");
  reset();
  cmd_rpm = 100;
  run(10);
  CHECK(moving());
  CHECK(count("stall") == 0);

  wheels_follow = false; // ran into a wall
  run(0.9);
  CHECK(moving());
  run(0.3);
  CHECK(!moving());
  CHECK(safetyMotionBlocked());
  CHECK(logged("SAFETY: stall both"));
  CHECK(brake_calls == 2);

  run(1.5); // commands are ignored during hold
  CHECK(!moving());
  run(0.7); // hold is over, PC still pushes forward
  CHECK(moving());
  run(1.2); // stalls again, no false trigger right after the hold
  CHECK(!moving());
  CHECK(count("stall") == 2);

  // Backing off after the hold works
  wheels_follow = true;
  cmd_rpm = -60;
  run(5);
  CHECK(moving());
  CHECK(count("stall") == 2);

  // Changing direction on the move is not a stall
  reset();
  cmd_rpm = 100;
  run(3);
  cmd_rpm = -100;
  run(3);
  cmd_rpm = 20;
  run(3);
  CHECK(moving());
  CHECK(count("stall") == 0);

  // Slowest crawl above rpm_min is not a stall
  reset();
  cmd_rpm = 5;
  run(10);
  CHECK(count("stall") == 0);

  // One wheel blocked
  reset();
  cmd_rpm = 100;
  run(2);
  for (int i = 0; i < 150; i++) {
    double left_enc = motorLeft.enc;
    step();
    motorLeft.enc = left_enc;
  }
  CHECK(logged("SAFETY: stall left"));
  CHECK(!moving());

  // Encoder wired backwards: PID would run away
  reset();
  encoder_reversed = true;
  cmd_rpm = 100;
  run(1.2);
  CHECK(!moving());
  CHECK(logged("SAFETY: stall both"));

  // PWM below pwm_min: PID is still winding up, not a stall yet
  reset();
  cfg.safety_stall_pwm_min = 0.6f;
  wheels_follow = false;
  cmd_rpm = 100;
  run(5);
  CHECK(moving());
  CHECK(count("stall") == 0);
}

void testBattery() {
  printf("battery\n");
  reset();
  cmd_rpm = 100;
  run(5);
  CHECK(safety_bat_state == SAFETY_BAT_OK);
  CHECK(moving());

  fake_battery_mv = 13900;
  run(2);
  CHECK(safety_bat_state == SAFETY_BAT_OK); // not long enough yet
  run(3);
  CHECK(safety_bat_state == SAFETY_BAT_LOW);
  CHECK(logged("SAFETY: battery low"));
  CHECK(moving()); // low still drives

  fake_battery_mv = 12000; // short sag under load
  run(1);
  fake_battery_mv = 13900;
  run(5);
  CHECK(safety_bat_state == SAFETY_BAT_LOW);

  fake_battery_mv = 13100;
  run(5);
  CHECK(safety_bat_state == SAFETY_BAT_STOP);
  CHECK(!moving());
  CHECK(!lidar_stub.running);
  CHECK(logged("SAFETY: battery stop"));

  fake_battery_mv = 13500; // rebounds with the load off: must not resume
  run(10);
  CHECK(safety_bat_state == SAFETY_BAT_STOP);
  CHECK(!moving());

  fake_battery_mv = 13800; // above stop + hysteresis
  run(5);
  CHECK(safety_bat_state == SAFETY_BAT_LOW);
  CHECK(lidar_stub.running);
  CHECK(moving());

  fake_battery_mv = 16500; // charged
  run(5);
  CHECK(safety_bat_state == SAFETY_BAT_OK);

  fake_battery_mv = 12700;
  bool slept = false;
  try {
    run(10);
  } catch (DeepSleepEntered &) {
    slept = true;
  }
  CHECK(slept);
  CHECK(safety_bat_sleeping);
  CHECK(!moving());
  CHECK(!lidar_stub.running);
  CHECK(logged("SAFETY: battery sleep"));

  // No battery, USB power on the bench: no limits
  reset();
  fake_battery_mv = 300;
  cmd_rpm = 100;
  run(10);
  CHECK(safety_bat_state == SAFETY_BAT_ABSENT);
  CHECK(moving());
  CHECK(count("stall") == 0);

  // Battery protection switched off in config
  reset();
  cfg.safety_bat_low = cfg.safety_bat_stop = cfg.safety_bat_sleep = 0;
  fake_battery_mv = 11000;
  cmd_rpm = 100;
  run(10);
  CHECK(moving());
}

bool setupSleeps() {
  try {
    safetySetup();
  } catch (DeepSleepEntered &) {
    return true;
  }
  return false;
}

void testBoot() {
  printf("boot check\n");
  reset();
  fake_battery_mv = 15000;
  CHECK(!setupSleeps());
  CHECK(safety_bat_state == SAFETY_BAT_OK);

  reset();
  fake_battery_mv = 13000;
  CHECK(!setupSleeps());
  CHECK(safety_bat_state == SAFETY_BAT_STOP);
  CHECK(safetyLidarBlocked());

  reset();
  fake_battery_mv = 12500; // cold boot on an empty battery
  CHECK(setupSleeps());

  reset();
  safety_bat_sleeping = true; // timer wake-up, battery only rebounded
  fake_battery_mv = 13100;
  CHECK(setupSleeps());
  CHECK(safety_bat_sleeping);

  reset();
  safety_bat_sleeping = true; // timer wake-up, charged
  fake_battery_mv = 16000;
  CHECK(!setupSleeps());
  CHECK(!safety_bat_sleeping);
  CHECK(safety_bat_state == SAFETY_BAT_OK);

  reset();
  fake_battery_mv = 200; // USB only
  CHECK(!setupSleeps());
  CHECK(safety_bat_state == SAFETY_BAT_ABSENT);
}

int main() {
  testCmdVelWatchdog();
  testStall();
  testBattery();
  testBoot();
  printf(failures ? "FAILED: %d\n" : "ALL PASSED\n", failures);
  return failures ? 1 : 0;
}
