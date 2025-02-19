/*
 * Copyright (c) 2022-2023 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>

#include "cmsis_os2.h"
#include "cmsis_vio.h"
#include "main.h"
#include "sds_rec.h"

static sdsRecId_t  recId[7] = { 0 };
static uint8_t     recBuf[7][1000];
static uint8_t     tempBuf[80];
static const char *recName[7] = { "In-ch0",
                                  "In-ch1",
                                  "In-ch2",
                                  "In-ch3",
                                  "In-ch4",
                                  "In-ch5",
                                  "Out"   };
static osThreadId_t threadTestDataId;

// Idle time counter in ms
uint32_t idle_ms = 0;

// External functions
extern int32_t socket_startup(void);

// Generator thread for simulated data
static __NO_RETURN void threadTestData(void *argument) {
  uint32_t num, buf_size;
  uint32_t timestamp;
  int32_t i, j;
  (void)argument;

  timestamp = osKernelGetTickCount();
  for (;;) {
    // Poll for thread termination request
    if (osThreadFlagsWait(0x01, osFlagsWaitAny, 0) == 0x01) {
      threadTestDataId = NULL;
      osThreadExit();
    }
    for (i = 0; i < 7; i++) {
      // Generate dummy sensor data
      buf_size = (i < 6) ? 60 : 40; 
      for (j = 0; j < buf_size; j++) {
        tempBuf[j] = (i + j) & 0xFF;
      }
      num = sdsRecWrite(recId[i], timestamp, tempBuf, buf_size);
      if (num != buf_size) {
        printf("%s: Recorder write failed\r\n", recName[i]);
      }
      osThreadYield();
    }
    timestamp += 10U;
    osDelayUntil(timestamp);
  }
}

// Start recording
static void recorder_start(void) {
  int32_t i;

  for (i = 0; i < 7; i++) {
    recId[i] = sdsRecOpen(recName[i], recBuf[i], sizeof(recBuf[i]), (i < 6) ? 15*60 : 15*40);
  }
  // Create data thread
  threadTestDataId = osThreadNew(threadTestData, NULL, NULL);

  printf("Recording started\r\n");
}

// Stop recording
static void recorder_stop(void) {
  int32_t i;

  // Send signal to data thread to self-terminate
  osThreadFlagsSet(threadTestDataId, 0x01);

  for (i = 0; i < 7; i++) {
    sdsRecClose(recId[i]);
    recId[i] = NULL;
  }

  printf("Recording stopped\r\n");
}

// Demo
static __NO_RETURN void demo(void *argument) {
  uint32_t value, value_last = 0U;
  uint8_t rec_active = 0U;
  int32_t i = 0;

  (void)argument;

  printf("Starting SDS recorder...\r\n");

  if (socket_startup() != 0) {
    printf("Socket startup failed\r\n");
    osThreadExit();
  }

  // Initialize recorder
  sdsRecInit(NULL);

  for (;;) {
    // Monitor user button
    value = vioGetSignal(vioBUTTON0);
    if (value != value_last) {
      value_last = value;
      if (value == vioBUTTON0) {
        // Button pressed
        if (rec_active == 0U) {
          rec_active = 1U;
          recorder_start();
        }
        else {
          rec_active = 0U;
          recorder_stop();
        }
      }
      osDelay(500U);
    }
    osDelay(100U);
    // Print system and idle time
    if (++i == 50) {
      uint32_t tick = osKernelGetTickCount();
      printf("Time: %d.%03d, idle: %d.%03d\r\n", tick/1000, tick%1000, idle_ms/1000, idle_ms%1000);
      i = 0;
    }
  }
}

// Measure idle time
__NO_RETURN void osRtxIdleThread(void *argument) {
  (void)argument;
  uint32_t ticks, prev;

  prev = osKernelGetTickCount();
  for (;;) {
    __WFI();
    ticks = osKernelGetTickCount();
    if (ticks == (prev + 1)) {
      // Count only full idle tick intervals
      idle_ms++;
    }
    prev = ticks;
  }
}

// Application initialize
int32_t app_main(void) {
  osKernelInitialize();
  osThreadNew(demo, NULL, NULL);
  osKernelStart();
  return 0;
}
