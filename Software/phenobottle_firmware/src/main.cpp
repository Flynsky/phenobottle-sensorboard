/**
 * usefull commands:
 * Compile, flash and monitor:
 * ❯ pio run --target upload --target monitor
 * clean
 * ❯ pio run --target fullclean --environment phenobottle_firmware
 * Generate compile_commands.json (for clang-uml/neovim or simular):
 * ❯ pio run -t compiledb
 * List installed Libaries
 * ❯ platformio lib list
 * Install Libarie
 * ❯ platformio lib install "ArduinoJson"
 */
#include "ADC_ADS1115.h"
#include "data_handeling.h"
#include "imu.h"
#include "opt.h"
#include <Arduino.h>
/**
 * Serial is connected to UART0 and the USB_CDC,
 * boath recieve it.
 * */
Console consoleSerial{&Serial};

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.print("\e[33m<phenobottle_firmware running>\e[0m\n\
type \"?\" for help\n");

  setup_adc();
  setup_imu();
  setup_opt(); // call after IMU, as it uses the same I2C bus

  consoleSerial.printCommandPrompt(); // initial command prompt display
}

void loop() {
  consoleSerial.toggle_status_led();
  consoleSerial.checkForInput();
  consoleSerial.sendSensorData();
}
