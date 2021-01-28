///*
// * NTC-thermistor.c
// *
// *  Created on: Jan 14, 2021
// *      Author: Artur
// */
//#include "NTC-thermistor.h"
//#include <stdio.h>
//#include <string.h>
//#include <math.h>
//
//#define ADCREAD_VREFINT_ELEMENT_NO  4 //Number of element in ADCread array which stores value of internal voltage reference measurement
//#define ADC_BUF_LEN 4095 // Maximal value of ADCread
//#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFF7A2A)) //Raw data acquired at temperature of 30 °C VDDA = 3.3 V
//															  //ST did a measurement for us when manufacturing the chip,
//															  //using a precise external reference voltage on the VREF+ pin,
//				 	 	 	 	 	 	 	 	 	 	 	  //and stored the resulting ADC reading into the system memory
//
//#define VCAL 3.3	//The voltage used as external reference at calibration. Put as 330 instead 3.3V to avoid float, vmeasured would be in 10mV units
//
//#define _NTC_R_SERIES         10000.0f
//#define _NTC_R_NOMINAL        10000.0f
//#define _NTC_TEMP_NOMINAL     25.0f
//#define _NTC_ADC_MAX          4096 //  12bit
//#define _NTC_BETA             3950
//
//float Rntc = 0; 		//Resistance of the thermistor
//float temp = 0; 		//Calculated temperature in Celcius
//float vrefint = 0; 	//Reference internal voltage
//float vrefext = 0; 	//External voltage calculated based on vrefint
//float vmeas = 0; 	//Measured voltage in volts [V]
//
//float NTC3950_ADCToTemperature(uint16_t ADCread[], uint8_t NTCnumber)
//{
//	vrefint = VCAL * (*TEMP30_CAL_ADDR) / ADC_BUF_LEN;
//	vrefext = vrefint * ADC_BUF_LEN / ADCread[ADCREAD_VREFINT_ELEMENT_NO];
//	vmeas = vrefext * ADCread[NTCnumber] / ADC_BUF_LEN;
//
//	// Calculating Resistance of the NTC Vmeasured*10kOhms
//	Rntc= (vmeas*(float)_NTC_R_NOMINAL)/(vrefext-vmeas);
//
//	// Calculating Temperature
//	temp = Rntc/(float)_NTC_R_NOMINAL;
//	temp = logf(temp);
//	temp = temp/(float)_NTC_BETA;
//	temp += 1.0f / ((float)_NTC_TEMP_NOMINAL + 273.15f);
//	temp = 1.0f/temp;
//	temp -= 273.15f;
//
//	return temp;
//}
