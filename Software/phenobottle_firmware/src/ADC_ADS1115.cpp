#include "ADC_ADS1115.h"
#include "ADS1115_WE.h"
#include "pins.h"
ADS1115_WE adc = ADS1115_WE(&Wire, I2C_ADDRESS);

void setup_adc() {
  Wire.begin(PIN_I2C1_SDA , PIN_I2C1_SCL , 1000000);
  if (!adc.init()) {
    Serial.println("ADS1115 not connected!");
  }
  // adc.setVoltageRange_mV(ADS1115_RANGE_6144);
  adc.setConvRate(ADS1115_860_SPS);
}

float readChannel(ADS1115_MUX channel, ADS1115_RANGE range) {
  float voltage = 0.0;
  adc.setVoltageRange_mV(range);
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  // while (adc.isBusy())
  //   delay(0);
  // }
  voltage = adc.getResult_V();
  return voltage;
}

float readPH(const float Temp_C) {
  const float offset = 1.024;
  const float Faraday = 9.6485309 * 10000; // Faraday Constant
  const float R_const = 8.314510;          // universal gas constan
  const float ln10 = 2.30258509299;        // ln(10)
  float Temp_K = Temp_C + 273.15;

  /*Select ADC range to 2.048V*/

  float volt_meas = readChannel(ADS1015_COMP_1_GND, ADS1115_RANGE_2048);

  float ph = 7.0 + (volt_meas - offset) * Faraday / (R_const * Temp_K * ln10);
  return ph;
}

float readTIA() {
  // TODO make a function to quickly read the TIA:
  // one function to enter a mode with fixed gain and mux
  // one function to read the TIA as fast as possible, without reading the gain back
  // one function to exit this mode
  // perhaps set a global lock flag to prevent accidental calls to NTC and PH functions
  // also use IRQs if possible

  const float gain = 10 * 1000 * 1000;

  /*Select ADC range to 0.256V*/

  float volt_meas = readChannel(ADS1015_COMP_2_GND, ADS1115_RANGE_0256);
  float I_pd = volt_meas / gain;
  return I_pd;
}

/**
 * Reads out the temperature of one NTC thermistor.
 * Temperature range with current resistor values: 0.25 til 44.85°C
 * @return temperature value or -1000000 if NTC is not connected/vaulty
 */
float readNTC() {
  /*circuit design*/
  const float VCC_NTC = 3.0;     // reference voltage for the NTC Readout
  const float R_SERIES = 5200.0; // Fixed resistor value in ohms (10kΩ)
  const float R41 = 10000.0;
  const float R43 = 51000.0;
  const float R53 = 10000.0;

  /*thermistor stats*/
  const float NTC_B = 3435.0;   // Beta parameter
  const float NTC_T0 = 298.15;  // Reference temperature in Kelvin (25°C)
  const float NTC_R0 = 10000.0; // Resistance at reference temperature (10kΩ)

  /*calculations*/
  const float R43_paral_R41 = R41 * R43 / (R41 + R43);
  const float gain = 1.0 / (R43_paral_R41 / (R43_paral_R41 + R53));
  const float R41_paral_R41 = 0.5 * R41;
  const float voffset = VCC_NTC * R41_paral_R41 / (R41_paral_R41 + R43);

  /*Select ADC range to 2.048V*/
  // if (readChannel(ADS1015_COMP_0_GND, ADS1115_RANGE_2048)< ADC_MAX_READ *
  // 0.05) {
  //   // error_handler(ERROR_NO_NTC_CONNECTED);
  //   return -1000000;
  // }

  /*Read out ADC in a buffer and calculating adc_average*/
  int adc_buffer = 0;
  int nTimes = 1;
  for (int i = 0; i < nTimes; i++) {
    adc_buffer += readChannel(ADS1015_COMP_0_GND, ADS1115_RANGE_2048);
  }
  float voltage_adc = ((float)adc_buffer) / nTimes;

  // /*removing spikes in adc_buffer*/
  // float voltage_adc;
  // unsigned int counter_spikes_removed = 0;
  // for (int i = 0; i < nTimes; i++)
  // {
  //     // debugf(">error:%f\n",adc_average-adc_buffer[i]);
  //     if (adc_buffer[i] > (adc_average + spike_tolerance) &&
  //         adc_buffer[i] < (adc_average - spike_tolerance))
  //     // replaces adc_buffer values which dividate from the adc_average about
  //     more than spike_tolerance
  //     {
  //         adc_buffer[i] = adc_average;
  //         counter_spikes_removed++;
  //     }
  // }
  // debugf("spikes removed:%u from %u samples\n", counter_spikes_removed,
  // nTimes);

  // /*calculating new adc_average without spikes*/
  // float filtered_adc_average = 0;
  // for (int i = 0; i < nTimes; i++)
  // {
  //     filtered_adc_average += adc_buffer[i];
  // }
  // filtered_adc_average = filtered_adc_average / nTimes;

  /*Converts the adc voltage after the opamp circuit to the voltage on the ntc*/
  float volt_ntc = voltage_adc * (1.0 / gain) + voffset;

  /*Converts ADC value to a Resistance*/
  float resistance = (float)R_SERIES * (((float)VCC_NTC / volt_ntc) - 1);

  /*Calculate the temperature in Kelvin using the Steinhart–Hart equation*/
  float tempK = 1.0 / (1.0 / NTC_T0 + (1.0 / NTC_B) * log(resistance / NTC_R0));

  /*Convert temperature from Kelvin to Celsius*/
  float tempC = tempK - 273.15;
  // Serial.printf("VADC:%f VNTC:%f R:%f T:%f\n", voltage_adc, volt_ntc,
  // resistance, tempC);
  return tempC;
}
