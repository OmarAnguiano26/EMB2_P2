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

#define EVENT_MINUTE  (1 << 0)


/**Definition of the handles*/
SemaphoreHandle_t minutes_semphr;
SemaphoreHandle_t hours_semphr;
SemaphoreHandle_t g_mutex;

EventGroupHandle_t g_events;

QueueHandle_t xQueue;

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
    minutes_semphr = xSemaphoreCreateBinary();
    hours_semphr = xSemaphoreCreateBinary();
    g_mutex = xSemaphoreCreateMutex();
    g_events = xEventGroupCreate();
    xQueue = xQueueCreate(ELEMENTS, sizeof(time_msg_t*));

    /**Create task*/
    xTaskCreate(Generate_SCL_TASK, "SCL", configMINIMAL_STACK_SIZE, NULL, 3, NULL);

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

	}
}

void Generate_SDA_ADDRESS_TASK()
{
	for(;;)
	{

	}
}

void Generate_SDA_DATA_TASK()
{
	for(;;)
	{

	}
}


