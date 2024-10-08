/*---------------------------------------------------------------------------
 * Copyright (c) 2020-2024 Arm Limited (or its affiliates). All rights reserved.
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
 *---------------------------------------------------------------------------*/

#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"

#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif

#include <stdio.h>
//#include "cv/linear_filters.h"

extern int stdout_init(void);
extern void app_main(void *argument);

osThreadId_t app_main_tid;

/*---------------------------------------------------------------------------
 * main function
 *---------------------------------------------------------------------------*/
int main(void)
{
  SystemCoreClockUpdate();     // System Initialization

  stdout_init();               // Initialize STDOUT for printing
  printf("\r\n= App is started =\r\n");

#if defined(RTE_Compiler_EventRecorder) && \
  (defined(__MICROLIB) ||                \
   !(defined(RTE_CMSIS_RTOS2_RTX5) || defined(RTE_CMSIS_RTOS2_FreeRTOS)))
  EventRecorderInitialize(EventRecordAll, 1U);      // Initialize EventRecorder if present
#endif
   
  osKernelInitialize();        // Initialize CMSIS-RTOS2

  app_main_tid = osThreadNew(app_main, NULL, NULL); // Create application thread

  osKernelStart();             // Start RTOS scheduler

  for (;;) {}
}
