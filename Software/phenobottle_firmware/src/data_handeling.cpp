#include "data_handeling.h"
#include "ADC_ADS1115.h"
#include "ArduinoJson.h"
#include "HWCDC.h"
#include "esp32-hal.h"
#include "imu.h"
#include "opt.h"
#include "pins.h"
#include "util.h"
#include <Arduino.h>

#define ANALOG_WRITE_RES 255
#define PERCENTAGE_TO_DUTY_CYCLE(p) /* fixed point percentage (0-100) to 8-bit duty cycle (0-255) */ \
  (p <= 0 ? 0 : p >= 100 ? ANALOG_WRITE_RES                                                          \
                         : (int)(p * ANALOG_WRITE_RES / 100))

/**
 * Checks for Commands send to the console
 * Is non-blocking and ment to be called frequently
 * * */
void Console::checkForInput()
{
  if (stream->available())
  {
    this->handleInput();
  }
}

void Console::printCommandPrompt()
{
  // print a simple prompt to show the user that the console is ready for input
  stream->print("> ");
  stream->flush();
}

/*Decodes and executes Commands send to console*/
void Console::handleInput()
{
  String buffer = stream->readStringUntil('\n');
  // stream->printf("Received data: %s\n", UserRxBufferFS);

  /*decode message*/
  char command[4] = {0}; // Store the command (e.g., "/C")
  float param0 = -1, param1 = -1, param2 = -1,
        param3 = -1; // Up to 4 parameters

  /*Use sscanf to extract the command and up to 4 floats*/
  int num_params = sscanf(buffer.c_str(), "%s %f %f %f %f", command, &param0,
                          &param1, &param2, &param3);

  // Print the command and the parameters
  /* stream->printf("~rec:%s|%f|%f|%f|%f|\n", command, param0, param1, param2,
   * param3);*/

  // encodes 4 char in one int to be compared by switch case
  int com_encoded = (int)((command[0] << 24) | (command[1] << 16) |
                          (command[2] << 8) | command[3]);

  switch (com_encoded)
  {
    /**here are the executions of all the commands */
    // case (int)('d' << 24 | 'f' << 16 | 'u' << 8 | 0): {
  /*help*/
  case (int)('?' << 24 | 0):
  {
    stream->print("\e[33m\n<help console>\n\e[0m");
    stream->print("structure of commands:\n");
    stream->print(
        "[str command][whitespace][param0=-1][whitespace][param1=-1]..\n");
    stream->print("examples:\n");
    stream->print("\"m 1 0.1 \":Set Motor 1 to 0.1(aka 10 percent\n");
    stream->print("\"p -1 1 -1 1000\":Print TIA every 1000ms\n");
    stream->print("command list:\n");
    stream->print("-?|this help screen\n");
    stream->print("-m|[number][duty_%][freq=100k]:\n"
                  "  sets motor [number] at"
                  "  [duty_%] power and optional [freq]Hz.\n");
    stream->print("-p|[NTC/pH/oD=0][TIA=0][IMU=0][Delay_ms=500]:\n"
                  "  change sensor output.\n"
                  "  0 disables, 1 enables, -1 no-change for given\n"
                  "  output. [Delay_ms] is between two prints. Less prints\n"
                  "  increases the maximal frequency\n");
    stream->print("-a|[duty_%]: sets Ambient Led at duty_%\n");
    stream->print("-o|: measure optical density\n");
    stream->print("-h|: hammer hit, record data and swing hammer\n");
    stream->print("\n");
    break;
  }

  case ('o' << 24 | 0):
  {
    stream->print("\e[33m\n<measure optical density>\n\e[0m");
    stream->flush();
    measure_OD(); // blocking
    stream->print("\e[33m\n<finished measuring optical density>\n\e[0m");
    break;
  }

  case ('h' << 24 | 0):
  {
    stream->print("\e[33m\n<hammer hit>\n\e[0m");
    stream->flush();
    hammer_hit(); // blocking
    stream->print("\e[33m\n<finished hammer hit>\n\e[0m");
    break;
  }

    /*control recieving data stream*/
  case ('p' << 24 | 0):
  {
    if (param0 >= 0)
      config_print.NTC_PH_OD = (bool)param0;
    if (param1 >= 0)
      config_print.TIA = (bool)param1;
    if (param2 >= 0)
      config_print.IMU = (bool)param2;
    if (param3 >= 0)
      config_print.DELAY = (unsigned long)param3;

    config_print.OPT = true; // FIXME cannot control OPT prints, due to 4 parameter limit!

    break;
  }

    /*control motors*/
  case ('m' << 24 | 0):
  {
    analogWriteResolution(8);

    if (param2 > 1)
    {
      analogWriteFrequency(param2);
    }

    int duty = PERCENTAGE_TO_DUTY_CYCLE(param1);

    if ((int)param0 == 0)
    {
      stream->printf("set motor 0 to %i\n", duty);
      analogWrite(PIN_MOTOR0, duty);
      pwm_config.motor0_duty = duty;
    }
    if ((int)param0 == 1)
    {
      stream->printf("set motor 1 to %i\n", duty);
      analogWrite(PIN_MOTOR1, duty);
      pwm_config.motor1_duty = duty;
    }
    if ((int)param0 == 2)
    {
      stream->printf("set motor 2 to %i\n", duty);
      analogWrite(PIN_MOTOR2, duty);
      pwm_config.motor2_duty = duty;
    }
    break;
  }

    /*control ambient light*/
  case ('a' << 24 | 0):
  {
    analogWriteResolution(8);

    int duty = PERCENTAGE_TO_DUTY_CYCLE(param0);

    if ((int)param0 >= 0)
    {
      stream->printf("set ambient led to %i\n", duty);
      analogWrite(PIN_AMBIENT_LED, duty);
      pwm_config.ambient_led_duty = duty;
    }
    break;
  }

  default:
  {
    stream->printf("unknown commnad: %s\n", command);
    stream->print("use \"?\" for help\n");
    break;
  }
  }

  printCommandPrompt();
}

/**
 * Sends Sensor data specified by struct sendSensorDataConfig .
 * Is non-blocking and ment to be called frequently
 * */
void Console::sendSensorData() const
{
  static unsigned long timestamp = millis() + config_print.DELAY;

  if (millis() > timestamp)
  {
    timestamp = millis() + config_print.DELAY;

    // unsigned long timestamp_o = micros(); // for transimmion timing

    /*Output format*/
    JsonDocument doc;
    if (config_print.NTC_PH_OD | config_print.IMU | config_print.TIA)
    {
      doc["time"] = millis();
    }

    /*ADC*/
    if (config_print.NTC_PH_OD)
    {
      doc["NTC"] = readNTC();
      doc["pH_cal"] = readPH(readNTC());
      const float TEMPERATURE_ROOM = 25.0;
      doc["pH"] = readPH(TEMPERATURE_ROOM);
    }

    if (config_print.TIA)
    {
      doc["TIA"] = readTIA();
    }

    /*IMU*/
    if (config_print.IMU)
    {
      if (init_imu)
      {
        inv_imu_sensor_event_t imu_event;
        // Get last event
        IMU.getDataFromRegisters(imu_event);
        doc["imu"]["a"][0] = imu_event.accel[0];
        doc["imu"]["a"][1] = imu_event.accel[1];
        doc["imu"]["a"][2] = imu_event.accel[2];
        doc["imu"]["g"][0] = imu_event.gyro[0];
        doc["imu"]["g"][1] = imu_event.gyro[1];
        doc["imu"]["g"][2] = imu_event.gyro[2];
        doc["imu"]["t"] = imu_event.temperature;
      }
    }

    if (config_print.OPT)
    {
      // libarary name "lux" is wrong, should be "irradiance"
      if (!opt3002_left_queue.empty())
      {
        OPT3002 result = opt3002_left_queue.front();
        opt3002_left_queue.pop();
        doc["opt"]["left"] = result.lux;
      }

      if (!opt3002_right_queue.empty())
      {
        OPT3002 result = opt3002_right_queue.front();
        opt3002_right_queue.pop();
        doc["opt"]["right"] = result.lux;
      }
    }

    /*Send Data*/
    if (!doc.isNull())
    {
      serializeJson(doc, *stream);
      // serializeJsonPretty
      stream->print("\n");
    }

    /*Time Transmission*/
    // long int timestamp_a = micros();
    // stream->printf("t:%ims\n", (timestamp_a - timestamp_o) / 1000.0);
  }
}

/**
 * Toggle every DELAY_MS Seconds.
 */
void Console::toggle_status_led()
{
  const unsigned long DELAY_MS = 500;
  static char status = 2;
  static unsigned long timestamp = millis() + DELAY_MS;

  // setup condition
  if (status == 2)
  {
    pinMode(PIN_LED_STATUS, OUTPUT);
    status = 0;
  }

  // toggle LED
  if (millis() > timestamp)
  {
    timestamp = millis() + DELAY_MS;
    status = !status;
    digitalWrite(PIN_LED_STATUS, status);
  }
}

float Console::measure_OD()
{
  const float od_clear = 0.0; // TODO: calibrate, perhaps add addtional calibration command and store in NVS

  // turn OD LED on
  pwm_config.od_led_duty = 255; // 100% duty cycle
  analogWrite(PIN_OD_LED, pwm_config.od_led_duty);

  // wait for the LED to stabilize
  delay(1000); // must be over 800ms, to allow previous OPT conversion to finish

  const uint8_t AVG_COUNT = 20;

  float opt_left_values[AVG_COUNT] = {0};
  float opt_right_values[AVG_COUNT] = {0};

  unsigned long start_time = millis();
  const unsigned long MAX_DURATION = 5000; // 5 seconds

  uint8_t i_left = 0, i_right = 0;
  while (i_left < AVG_COUNT || i_right < AVG_COUNT)
  {
    if (i_left < AVG_COUNT && !opt3002_left_queue.empty())
    {
      OPT3002 result = opt3002_left_queue.front();
      opt3002_left_queue.pop();
      opt_left_values[i_left++] = result.lux;
    }

    if (i_right < AVG_COUNT && !opt3002_right_queue.empty())
    {
      OPT3002 result = opt3002_right_queue.front();
      opt3002_right_queue.pop();
      opt_right_values[i_right++] = result.lux;
    }

    if (millis() - start_time > MAX_DURATION)
    {
      stream->printf("Warning: Timeout while measuring OD");
      return 0.0; // return 0.0 if timeout
    }
  }

  float avg, avg_left = 0.0, avg_right = 0.0;
  for (uint8_t i = 0; i < AVG_COUNT; i++)
  {
    avg_left += opt_left_values[i];
    avg_right += opt_right_values[i];
  }
  avg_left /= AVG_COUNT;
  avg_right /= AVG_COUNT;
  avg = (avg_left + avg_right) / 2.0;

  // turn OD LED off
  pwm_config.od_led_duty = 0; // 0% duty cycle
  analogWrite(PIN_OD_LED, pwm_config.od_led_duty);

  // calculate OD
  float od = (avg - od_clear) / od_clear * 100.0;

  // print result
  stream->printf("OD: %.2f%%\n", od);
  stream->printf("IR left: %.2f, IR right: %.2f\n, IR avg: %.2f\n",
                 avg_left, avg_right, avg);

  return od;
}

void Console::hammer_hit()
{
  unsigned long start_time = millis();
  const unsigned long RECORD_DURATION = 5000; // 5 seconds

  // Initilize buffers for IMU, TIA and OPTs
  const uint32_t BUFFER_SIZE_IMU = RECORD_DURATION * 10;
  const uint32_t BUFFER_SIZE_OPT = RECORD_DURATION * 1;
  const uint32_t BUFFER_SIZE_TIA = RECORD_DURATION * 20;

  float *imu_mag_buffer;
  float *tia_buffer;
  float *opt_left_buffer;
  float *opt_right_buffer;

  // dynamic allocation on stack
  // TODO move this into static memory as it re-boots at runtime if the buffers are too large
  imu_mag_buffer = (float *)malloc(BUFFER_SIZE_IMU * sizeof(float));
  tia_buffer = (float *)malloc(BUFFER_SIZE_TIA * sizeof(float));
  opt_left_buffer = (float *)malloc(BUFFER_SIZE_OPT * sizeof(float));
  opt_right_buffer = (float *)malloc(BUFFER_SIZE_OPT * sizeof(float));

  // sanity check
  if (!imu_mag_buffer || !tia_buffer || !opt_left_buffer || !opt_right_buffer)
  {
    // might not be printed if the buffers are too large, as stack can be smashed atp
    stream->printf("Error: Not enough memory for buffers\n");
    return;
  }

  uint32_t imu_index = 0;
  uint32_t tia_index = 0;
  uint32_t opt_left_index = 0;
  uint32_t opt_right_index = 0;

  // Turn off ambient light and OD LED
  analogWrite(PIN_AMBIENT_LED, 0);
  analogWrite(PIN_OD_LED, 0);

  analogWrite(PIN_MOTOR1, 255); // turn on motor 0 to swing hammer

  stream->printf("\e[33m\n<Recording started>\n\e[0m");

  do
  {
    // TODO read IMU and TIA based off of interrupt, to not block the loop

    if (init_imu && imu_index < BUFFER_SIZE_IMU)
    {
      inv_imu_sensor_event_t imu_event;
      // Get last event
      IMU.getDataFromRegisters(imu_event);
      float imu_acc_mag = sqrt(imu_event.accel[0] * imu_event.accel[0] +
                               imu_event.accel[1] * imu_event.accel[1] +
                               imu_event.accel[2] * imu_event.accel[2]);
      imu_mag_buffer[++imu_index] = imu_acc_mag / 2048.0; // scale to g // TODO make this variable/constant
    }

    if (tia_index < BUFFER_SIZE_TIA)
    {
      tia_buffer[tia_index++] = readTIA();
    }

    if (opt_left_index < BUFFER_SIZE_OPT)
    {
      if (!opt3002_left_queue.empty())
      {
        OPT3002 result = opt3002_left_queue.front();
        opt3002_left_queue.pop();
        opt_left_buffer[opt_left_index++] = result.lux;
      }
    }

    if (opt_right_index < BUFFER_SIZE_OPT)
    {
      if (!opt3002_right_queue.empty())
      {
        OPT3002 result = opt3002_right_queue.front();
        opt3002_right_queue.pop();
        opt_right_buffer[opt_right_index++] = result.lux;
      }
    }

  } while ((millis() - start_time) < RECORD_DURATION);

  analogWrite(PIN_MOTOR1, 0); // turn off motor 0
  // restore ambient light and OD LED
  analogWrite(PIN_AMBIENT_LED, pwm_config.ambient_led_duty);
  analogWrite(PIN_OD_LED, pwm_config.od_led_duty);

  // 6 print data
  stream->printf("\e[33m\n<Recording stopped>\n\e[0m");

  stream->printf("\nIMU data\n");
  plotBuffer(imu_mag_buffer, imu_index, stream);

  stream->printf("\nTIA data\n");
  plotBuffer(tia_buffer, tia_index, stream);

  stream->printf("\nOPT left data\n");
  plotBuffer(opt_left_buffer, opt_left_index, stream);

  stream->printf("\nOPT right data\n");
  plotBuffer(opt_right_buffer, opt_right_index, stream);

  free(imu_mag_buffer);
  free(tia_buffer);
  free(opt_left_buffer);
  free(opt_right_buffer);
}
