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

#include "RTE_Components.h"

#include "cmsis_os2.h"
#include "os_tick.h"
#include "cmsis_vio.h"

#include "main.h"
#include "sds_rec.h"

// Configuration
#ifndef REC_BUF_SIZE_IMU_IN
#define REC_BUF_SIZE_IMU_IN                 8192U
#endif
#ifndef REC_IO_THRESHOLD_IMU_IN
#define REC_IO_THRESHOLD_IMU_IN             7400U
#endif

#ifndef REC_BUF_SIZE_ML_OUT
#define REC_BUF_SIZE_ML_OUT                 1536U
#endif
#ifndef REC_IO_THRESHOLD_ML_OUT
#define REC_IO_THRESHOLD_ML_OUT             1400U
#endif

// Error handling
struct {
  uint8_t  occurred;
  uint8_t  reported;
  char    *file;
  uint32_t line;
} sds_error;

#define sds_assert(cond) if (!(cond) && (!sds_error.occurred)) { \
  sds_error.occurred = 1; sds_error.file = __FILE__; sds_error.line = __LINE__; }

static uint8_t rec_buf_in [REC_BUF_SIZE_IMU_IN];
static uint8_t rec_buf_out[REC_BUF_SIZE_ML_OUT];
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

// ML output buffer
struct OUT {
  struct {
    uint16_t x;
    uint16_t y;
  } out;
} ml_buf[10];

// Idle time counter in ms
uint32_t cnt_idle = 0;

// External functions
extern int32_t socket_startup(void);
__WEAK int32_t socket_startup(void) {
  return 0;
}

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
    ml_buf[i].out.x = val;
    ml_buf[i].out.y = val % 500;
  }
  index_out = (index_out + i) % 1000;
}

// Print CPU usage in %
static void cpu_usage(int32_t active) {
  static uint32_t cnt;

  if (!active) {
    cnt_idle = cnt = 0;
  }
  else if (++cnt >= 30) {
    // Usage print interval is 3 seconds
    printf("CPU usage: %.2f%%\r\n", (float)(48000 - cnt_idle) / 480.0);
    cnt_idle = cnt = 0;
  }
}

// Thread for generating simulated data
static __NO_RETURN void threadTestData(void *argument) {
  sdsRecId_t *in, *out;
  uint32_t n, timestamp;
  (void)argument;

  in  = sdsRecOpen("In", rec_buf_in, sizeof(rec_buf_in), REC_IO_THRESHOLD_IMU_IN);
  out = sdsRecOpen("Out", rec_buf_out, sizeof(rec_buf_out),REC_IO_THRESHOLD_ML_OUT);

  timestamp = osKernelGetTickCount();
  for (;;) {
    if (stop_req) {
      sdsRecClose(in);
      sdsRecClose(out);

      stop_req = 0;
      osThreadExit();
    }

    CreateTestData();

    n = sdsRecWrite(in, timestamp, &imu_buf, sizeof(imu_buf));
    sds_assert(n == sizeof(imu_buf));

    n = sdsRecWrite(out, timestamp, &ml_buf, sizeof(ml_buf));
    sds_assert(n == sizeof(ml_buf));

    timestamp += 10U;
    osDelayUntil(timestamp);
  }
}

// Demo thread
static __NO_RETURN void demo(void *argument) {
  uint32_t state  = 0;
  int32_t  active = 0;
  (void)argument;

  printf("Starting SDS recorder...\n");

  if (socket_startup() != 0) {
    printf("Socket startup failed\n");
    osThreadExit();
  }

  // Initialize recorder
  sdsRecInit(NULL);

  for (;;) {
    // BUTTON0 toggles recording on/off
    if (state != vioGetSignal(vioBUTTON0)) {
      state ^= vioBUTTON0;
      if (state == vioBUTTON0) {
        if (!active) {
          osThreadNew(threadTestData, NULL, NULL);
          printf("Recording started\n");
        }
        else {
          stop_req = 1;    
          printf("Recording stopped\n");
        }
        active ^= 1;
      }
    }
    if (sds_error.occurred) {
      printf("SDS error in file: %s line %d\n", sds_error.file, sds_error.line);
      sds_error.reported = 1;
      sds_error.occurred = 0;
    }
    osDelay(100U);
    cpu_usage(active);
  }
}

// Measure system idle time
__NO_RETURN void osRtxIdleThread(void *argument) {
  uint32_t tick, next = 0xFFFFFFFF;
  (void)argument;

  for (;;) {
    __WFI();
    tick = osKernelGetTickCount();
    if (tick == next) {
      // Counts in sixteenths of an interval
      cnt_idle += (16 - 16*OS_Tick_GetCount()/OS_Tick_GetInterval());
    }
    next = tick + 1;
  }
}

// Application initialize
int32_t app_main(void) {
  osKernelInitialize();
  osThreadNew(demo, NULL, NULL);
  osKernelStart();
  return 0;
}
