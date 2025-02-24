/*
 * Copyright (c) 2022-2025 Arm Limited. All rights reserved.
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

static uint8_t rec_ibuf[1500];
static uint8_t rec_obuf[500];
static int32_t stop_req = 0;

// IMU sensor buffer
struct IMU {
  struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
  } accelerometer;
  struct  {
    uint16_t x;
    uint16_t y;
    uint16_t z;
  } gyroscpe;
} imu_buf[30];

// Output ML buffer
struct OUT {
  struct {
    uint16_t x;
    uint16_t y;
  } ml;
} out_buf[10];

// Idle time counter in ms
uint32_t idle_ms = 0;

// External functions
extern int32_t socket_startup(void);

// Create dummy test data
static void CreateTestData () {
  static uint16_t index_in, index_out;
  uint16_t val;
  int32_t i;

  // Sensor input data
  for (i = 0; i < 30; i++) {
    val = (index_in + i) % 3000;
    imu_buf[i].accelerometer.x = val;
    val = (val + 250) % 3000;
    imu_buf[i].accelerometer.y = 2999 - val;
    val = (val + 300) % 3000;
    imu_buf[i].accelerometer.z = (val < 1500) ? val : (2999 - val);

    val = (val + 150) % 1500;
    imu_buf[i].gyroscpe.x = val;
    val = (val + 70) % 1500;
    imu_buf[i].gyroscpe.y = 1499 - val;
    val = (val + 120) % 1500;
    imu_buf[i].gyroscpe.z = (val < 750) ? val : (1499 - val);
  }
  index_in = (index_in + i) % 3000;

  // ML output data
  for (i = 0; i < 10; i++) {
    val = (index_out + i) % 1000;
    out_buf[i].ml.x = val;
    out_buf[i].ml.y = val % 500;
  }
  index_out = (index_out + i) % 1000;
}

// CPU usage (in %)
static void cpu_usage(void) {
  static uint32_t cnt;
  float usage;

  if ((++cnt % 50) == 0) {
    uint32_t tick = osKernelGetTickCount();
    usage = ((float)(tick - idle_ms) * 100) / tick;
    printf("CPU Time: %.1fs, usage: %.2f%%\r\n", (float)tick/1000, usage);
  }
}

// Generator thread for simulated data
static __NO_RETURN void threadTestData(void *argument) {
  sdsRecId_t *in, *out;
  uint32_t n, timestamp;
  int32_t i;
  (void)argument;

  in  = sdsRecOpen("In", rec_ibuf, sizeof(rec_ibuf), 3*(sizeof(imu_buf)+8));
  out = sdsRecOpen("Out", rec_obuf, sizeof(rec_obuf), 10*(sizeof(out_buf)+8));

  printf("Recording started\r\n");
  timestamp = osKernelGetTickCount();
  for (;;) {
    if (stop_req) {
      sdsRecClose(in);
      sdsRecClose(out);

      printf("Recording stopped\r\n");
      stop_req = 0;
      osThreadExit();
    }

    CreateTestData();
    n = sdsRecWrite(in, timestamp, &imu_buf, sizeof(imu_buf));
    if (n != sizeof(imu_buf)) {
      printf("In: Recorder write failed\r\n");
    }
    n = sdsRecWrite(out, timestamp, &out_buf, sizeof(out_buf));
    if (n != sizeof(out_buf)) {
      printf("Out: Recorder write failed\r\n");
    }
    timestamp += 10U;
    osDelayUntil(timestamp);
  }
}

// Demo task
static __NO_RETURN void demo(void *argument) {
  uint32_t state = 0;
  int32_t  active = 0;
  (void)argument;

  printf("Starting SDS recorder...\r\n");

  if (socket_startup() != 0) {
    printf("Socket startup failed\r\n");
    osThreadExit();
  }

  // Initialize recorder
  sdsRecInit(NULL);

  for (;;) {
    // BUTTON0 toggles recording on/off
    if (state != vioGetSignal(vioBUTTON0)) {
      state ^= vioBUTTON0;
      if (state == vioBUTTON0) {
        if (!active) osThreadNew(threadTestData, NULL, NULL);
        else         stop_req = 1;    
        active ^= 1;
      }
    }
    osDelay(100U);
    cpu_usage();
  }
}

// Measure system idle time
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
