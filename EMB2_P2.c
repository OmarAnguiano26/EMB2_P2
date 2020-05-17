/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    EM2_T03.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
/* TODO: insert other definitions and declarations here. */

#define I2C_SDA_PORT			(PORTE)
#define I2C_SCL_PORT 			(PORTE)
#define I2C_SDA_GPIO			(GPIOE)
#define I2C_SCL_GPIO 			(GPIOE)
#define I2C_SDA_PIN 			(25U)
#define I2C_SCL_PIN 			(24U)

#define EVENT_MINUTE  			(1 << 0)
#define ELEMENTS				(3U)
#define FIVE_MS					(5u)
#define HIGH					(1U)
#define LOW						(0U)

typedef enum{ZERO,ONE,TWO,THREE,FOUR,FIVE,SIX,SEVEN,EIGHT,NINE,TEN,ELEVEN,TWELVE,THIRTEEN,
	FOURTEEN,FIFTEEN,SIXTEEN}num_t;

/**Definition of the handles*/
SemaphoreHandle_t i2c_semphr;
SemaphoreHandle_t i2c_data_semphr;
SemaphoreHandle_t g_mutex;

EventGroupHandle_t g_events;

QueueHandle_t xQueue;

uint8_t counter = ZERO;
uint8_t data_counter = ZERO;
uint8_t data = 0xAA;
uint8_t data_2;

/**Task definitions*/
	void Generate_SCL_TASK();
	void Generate_SDA_ADDRESS_TASK();
	void Generate_SDA_DATA_TASK();
/*
 *
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();
    PRINTF("START\n");


    /**Creates semphr, events and mutex*/
    i2c_semphr = xSemaphoreCreateBinary();
    i2c_data_semphr = xSemaphoreCreateBinary();
    g_mutex = xSemaphoreCreateMutex();
    g_events = xEventGroupCreate();

    /**Configs the GPIO*/
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {ZERO};

      /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortE);
    PORT_SetPinConfig(I2C_SCL_PORT, I2C_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_SDA_PORT, I2C_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_SCL_GPIO, I2C_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_SDA_GPIO, I2C_SDA_PIN, &pin_config);
    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, LOW);


    /**Create task*/
    xTaskCreate(Generate_SCL_TASK, "SCL", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(Generate_SDA_ADDRESS_TASK, "SDA_ADDRESS", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(Generate_SDA_DATA_TASK, "SDA", configMINIMAL_STACK_SIZE, NULL, 3, NULL);

    vTaskStartScheduler();

    while(1)
    {

    }
    return 0 ;
}

void Generate_SCL_TASK()
{
	for(;;)
	{
		if (8 > counter)
		{
			GPIO_PinWrite(I2C_SCL_GPIO, I2C_SCL_PIN, LOW);
			vTaskDelay(pdMS_TO_TICKS(FIVE_MS));
			xSemaphoreGive(i2c_semphr);
			GPIO_PinWrite(I2C_SCL_GPIO, I2C_SCL_PIN, HIGH);
			vTaskDelay(pdMS_TO_TICKS(FIVE_MS));
			counter ++;
			if (counter == 8)
			{
				vTaskDelay(pdMS_TO_TICKS(50));
				counter++;
			}
		}
		if(8 < counter && 17 > counter)
		{
			GPIO_PinWrite(I2C_SCL_GPIO, I2C_SCL_PIN, LOW);
			vTaskDelay(pdMS_TO_TICKS(FIVE_MS));
			xSemaphoreGive(i2c_data_semphr);
			GPIO_PinWrite(I2C_SCL_GPIO, I2C_SCL_PIN, HIGH);
			vTaskDelay(pdMS_TO_TICKS(FIVE_MS));
			counter ++;
			if(17 == counter)
			{
				vTaskDelay(pdMS_TO_TICKS(100));
			}
		}

	}
}

void Generate_SDA_ADDRESS_TASK()
{
	for(;;)
	{
		xSemaphoreTake(i2c_semphr,portMAX_DELAY);
		switch (counter)
		{
			case ZERO:
			    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, HIGH);
				break;
			case ONE:
			    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, HIGH);
				break;
			case TWO:
			    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, HIGH);
				break;
			case THREE:
			    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, LOW);
				break;
			case FOUR:
			    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, LOW);
				break;
			case FIVE:
			    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, HIGH);
				break;
			case SIX:
			    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, LOW);
				break;
			case SEVEN:
			    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, HIGH);
				break;
			default:
				break;
		}
	}
}

void Generate_SDA_DATA_TASK()
{
	for(;;)
	{
		xSemaphoreTake(i2c_data_semphr,portMAX_DELAY);
		switch (counter)
				{
					case NINE:
					    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, LOW);
						break;
					case TEN:
					    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, LOW);
						break;
					case ELEVEN:
					    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, LOW);
						break;
					case TWELVE:
					    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, HIGH);
						break;
					case THIRTEEN:
					    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, HIGH);
						break;
					case FOURTEEN:
					    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, LOW);
						break;
					case FIFTEEN:
					    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, HIGH);
						break;
					case SIXTEEN:
					    GPIO_PinWrite(I2C_SDA_GPIO, I2C_SDA_PIN, LOW);
						break;
					default:
						break;
				}
	}
}


