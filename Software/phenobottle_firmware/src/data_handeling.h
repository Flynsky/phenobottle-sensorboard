#pragma once
#include <Arduino.h>

struct sendSensorDataConfig {
  bool NTC_PH_OD = false;
  bool IMU = false;
  bool TIA = false;
  bool OPT = false; // Optical sensor
  unsigned long DELAY = 500; // ms
};

typedef struct {
  int motor0_duty = 0;
  int motor1_duty = 0;
  int motor2_duty = 0;
  int ambient_led_duty = 0;
  int od_led_duty = 0;
} pwmConfig_t;

class Console {
private:
  Stream *stream;
  sendSensorDataConfig config_print;
  void handleInput();

  pwmConfig_t pwm_config;
public:
  explicit Console(Stream *s = &Serial) : stream(s) {};
  ~Console()=default;
  void checkForInput();
  void printCommandPrompt();
  void sendSensorData() const;
  void toggle_status_led();
  float measure_OD();
  void hammer_hit();
};
