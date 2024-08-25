/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "sub_functions.h"
#include "main.h"


extern bool ITM_debug;


//code 

void ITM_SendString(const char *str) {
    while (*str && ITM_debug) {
        ITM_SendChar(*str++);
    }
}

void type_conversion01(adc_channel_data *Nutzdaten,  uint8_t outputArray[8]) {

    // Zerlegen von value1 in 4 Bytes
    outputArray[0] = (Nutzdaten->channel0 >> 24) & 0xFF;  // Höchstwertiges Byte von value1
    outputArray[1] = (Nutzdaten->channel0 >> 16) & 0xFF;
    outputArray[2] = (Nutzdaten->channel0 >> 8) & 0xFF;
    outputArray[3] = Nutzdaten->channel0 & 0xFF;          // Niedrigstwertiges Byte von value1

    // Zerlegen von value2 in 4 Bytes
    outputArray[4] = (Nutzdaten->channel1 >> 24) & 0xFF;  // Höchstwertiges Byte von value2
    outputArray[5] = (Nutzdaten->channel1 >> 16) & 0xFF;
    outputArray[6] = (Nutzdaten->channel1 >> 8) & 0xFF;
    outputArray[7] = Nutzdaten->channel1 & 0xFF;          // Niedrigstwertiges Byte von value2
}



