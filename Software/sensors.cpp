// https://eu.mouser.com/ProductDetail/TDK/NTCGS163JF103FT8?qs=dbcCsuKDzFU4pk95bZlq7w%3D%3D&countryCode=DE&currencyCode=EUR
// https://www.mouser.de/ProductDetail/Amphenol-Advanced-Sensors/JI-103C1R2-L301?qs=JUmsgfbaopRXkasA8RUqKg%3D%3D&countryCode=DE&currencyCode=EUR

#include "sensors.h"
#include "math.h"

volatile char adc_init = 0;

void adc_setup()
{
    /*--*/
    adc_init = 1;
}

/**
 * @param NTC specify here wich NTC y wanna check. dont specify anything or set to 255 checks the currently connected one
 * @return 1 if connected, 0 if not
 *  */
uint8_t temp_isconnected()
{
    if (temp_read_one() == -1000000.0)
    {
        return 0;
    }
    return 1;
}

/**
 * Reads out the temperature of one NTC thermistor.
 * Temperature range with current resistor values: 0.25 til 44.85°C
 * @return temperature value or -1000000 if NTC is not connected/vaulty
 */
float temp_read_one()
{
    /*circuit design*/
    const float VCC_NTC = 3.0;     // reference voltage for the NTC Readout
    const float R_SERIES = 5200.0; // Fixed resistor value in ohms (10kΩ)
    const float R41 = 10000.0;
    const float R43 = 51000.0;
    const float R53 = 10000.0;

    /*thermistor stats*/
    const float NTC_B = 3435.0;      // Beta parameter
    const float NTC_T0 = 298.15;         // Reference temperature in Kelvin (25°C)
    const float NTC_R0 = 10000.0;        // Resistance at reference temperature (10kΩ)

    /*calculations*/
    const float R43_paral_R41 = R41 * R43 / (R41 + R43);
    const float gain = 1.0 / (R43_paral_R41 / (R43_paral_R41 + R53));
    const float R41_paral_R41 = 0.5 * R41;
    const float voffset = VCC_NTC * R41_paral_R41 / (R41_paral_R41 + R43);

    if (!adc_init)
    {
        adc_setup();
    }

    /*Select ADC range to 2.048V*/

    /* check if NTC connected*/
    if (analogRead(PIN_TEMPADC) < ADC_MAX_READ * 0.05)
    {
        // error_handler(ERROR_NO_NTC_CONNECTED);
        return -1000000;
    }

    /*Read out ADC in a buffer and calculating adc_average*/
    int adc_buffer = 0;
     for (int i = 0; i < nTimes; i++)
    {
        adc_buffer += analogRead(ADC0);
    }
    float adc_average = ((float)adc_buffer) / nTimes;

    // /*removing spikes in adc_buffer*/
    // float voltage_adc;
    // unsigned int counter_spikes_removed = 0;
    // for (int i = 0; i < nTimes; i++)
    // {
    //     // debugf(">error:%f\n",adc_average-adc_buffer[i]);
    //     if (adc_buffer[i] > (adc_average + spike_tolerance) &&
    //         adc_buffer[i] < (adc_average - spike_tolerance))
    //     // replaces adc_buffer values which dividate from the adc_average about more than spike_tolerance
    //     {
    //         adc_buffer[i] = adc_average;
    //         counter_spikes_removed++;
    //     }
    // }
    // debugf("spikes removed:%u from %u samples\n", counter_spikes_removed, nTimes);

    // /*calculating new adc_average without spikes*/
    // float filtered_adc_average = 0;
    // for (int i = 0; i < nTimes; i++)
    // {
    //     filtered_adc_average += adc_buffer[i];
    // }
    // filtered_adc_average = filtered_adc_average / nTimes;

    /*calculating volgae form adc value*/
    float voltage_adc = (adc_average / ADC_MAX_READ) * VCC_NTC;

    /*Converts the adc voltage after the opamp circuit to the voltage on the ntc*/
    float volt_ntc = voltage_adc * (1.0 / gain) + voffset;

    /*Converts ADC value to a Resistance*/
    float resistance = (float)R_SERIES * (((float)VCC_NTC / volt_ntc) - 1);

    /*Calculate the temperature in Kelvin using the Steinhart–Hart equation*/
    float tempK = 1.0 / (1.0 / NTC_T0 + (1.0 / NTC_B) * log(resistance / NTC_R0));

    /*Convert temperature from Kelvin to Celsius*/
    float tempC = tempK - 273.15;
    // debugf_info("VADC:%f VNTC:%f R:%f T:%f\n", voltage_adc, volt_ntc, resistance, tempC);
    return tempC;
}

float ph_read(const float Temp_C){
    const float offset = 1.024;
    const float Faraday = 9.6485309*10000; //Faraday Constant
    const float R_const =  8.314510; //universal gas constan
    const float ln10 = 2.30258509299;//ln(10)
    float Temp_K = Temp_C + 273.15;

    /*Select ADC range to 2.048V*/

    float volt_meas = analogRead(ADC1);

    float ph = 7.0 + (volt_meas-offset) *  Faraday /(R_const*Temp_K*ln10);
    return ph;
}

float tia_read(){
    const float gain = 10*1000*1000;

    /*Select ADC range to 0.256V*/

    float volt_meas = analogRead(ADC2);
    float I_pd=volt_meas/gain;
    return I_pd;
}

float update_cur_integrator(){
    const float CAP_0 = 100; //nF
    const float CAP_1 = 10; //nF

    static last_volt_meas = analogRead(ADC3);
    static last_timestamp = micros();

    /*Select ADC range to 4.096V*/
    float volt_meas = analogRead(ADC3);
    float timestamp = micros();

    /*i_pd calculation over 2 samples*/
    float delta_v = last_volt_meas - volt_meas;
    float delta_t = timestamp-last_timestamp;
    float i_photodiode=C*delta_v/delta_t;

    /*i_pd calculation over complete discharge*/

    return i_photodiode;
}




