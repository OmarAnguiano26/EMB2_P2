/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void sck_task(void *pvParameters);
static void sda_rd_task(void *pvParameters);
static void sda_send_task(void *pvParameters);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
typedef struct
{
    SemaphoreHandle_t addres_semphr;
    SemaphoreHandle_t data_semphr;
} parameters_task_t;

int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    static parameters_task_t parameters_task;
    parameters_task.addres_semphr = xSemaphoreCreateMutex();
    parameters_task.data_semphr = xSemaphoreCreateMutex();

    xTaskCreate(sck_task, "sck task", configMINIMAL_STACK_SIZE + 100, (void*)&parameters_task, configMAX_PRIORITIES, NULL);
    xTaskCreate(sda_rd_task, "sda_rd_task", configMINIMAL_STACK_SIZE + 100, (void*)&parameters_task, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(sda_send_task, "sda_send_task", configMINIMAL_STACK_SIZE + 100, (void*)&parameters_task, configMAX_PRIORITIES-1, NULL);

    vTaskStartScheduler();
    for (;;){
}
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */


static void sck_task(void *pvParameters)
{
	uint8_t value;
	uint8_t tmp = 0;
	parameters_task_t parameters_task = *((parameters_task_t*)pvParameters);
    for (;;)
    {

    	if(tmp<8)
    	{
        	vTaskDelay(50);
        //    PRINTF("-");
        	value = 1;
        	xSemaphoreGive(parameters_task.addres_semphr);
        	vTaskDelay(50);
        //    PRINTF("_");
        	value = 0;
        	tmp++;
    	}
    }
}

static void sda_rd_task(void *pvParameters)
{
	uint8_t address = 0b10101010;

	parameters_task_t parameters_task = *((parameters_task_t*)pvParameters);
    for (;;)
    {
    	xSemaphoreTake(parameters_task.addres_semphr, portMAX_DELAY);
        PRINTF("%i",address);
        xSemaphoreGive(parameters_task.data_semphr);
    }
}

static void sda_send_task(void *pvParameters)
{
	uint8_t data = 0b11000011;

	parameters_task_t parameters_task = *((parameters_task_t*)pvParameters);
    for (;;)
    {
    	xSemaphoreTake(parameters_task.addres_semphr, portMAX_DELAY);
        PRINTF("%i",data);
    }
}

