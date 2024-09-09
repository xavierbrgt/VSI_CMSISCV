/* Copyright 2022 Arm Limited. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/


#ifdef _RTE_
#include "RTE_Components.h"
#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#else
#define EventStartCv(slot, v1, v2)
#define EventStopCv(slot, v1, v2)
#endif
#endif

#include CMSIS_device_header  // Device-specific defines and CMSIS-Core
#include "cmsis_os2.h"        // CMSIS-RTOS2 API
#include "micro_logger.h"     // Application logging engine to UART
#include "video_drv.h"        // Video Driver API
#include "hal.h"              // Device HAL, here for LCD access

#include "cv/linear_filters.h"
#include <stdlib.h>
#include "cv/feature_detection.h"
/* Video input characteristics */
#define COLOR_BLACK  0
#define IMAGE_WIDTH (192U)
#define IMAGE_HEIGHT (192U)
#define CHANNELS_IMAGE_DISPLAYED (3U)
#define IMAGE_DATA_SIZE (IMAGE_WIDTH*IMAGE_HEIGHT*CHANNELS_IMAGE_DISPLAYED)
#define FRAME_RATE (30U)

//#define INPUT_IMAGE "./samples/typing.mp4"  // Input file path
#define INPUT_IMAGE "./samples/glasses.bmp"  // Input file path
#define OUTPUT_IMAGE "./samples/glassesout.png"  // Input file path

static uint8_t ImageBuf[IMAGE_DATA_SIZE];   // Buffer for holding an input frame
static uint8_t ImageBufOut[IMAGE_DATA_SIZE];
/*---------------------------------------------------------------------------
 * User application initialization
 *---------------------------------------------------------------------------*/
void app_init()
{
  /* Initializing video driver */
  if (VideoDrv_Initialize(NULL) != VIDEO_DRV_OK) {
    log_error("Failed to initialise video driver\n");
  }
}

/*---------------------------------------------------------------------------
 * User application run
 *---------------------------------------------------------------------------*/
void app_run()
{
  void* imgFrame = NULL;
  void* outFrame = NULL;
  /* Video coordinates on LCD */
  uint32_t dataPsnImgDownscaleFactor = 1;
  uint32_t dataPsnImgStartX          = 10;
  uint32_t dataPsnImgStartY          = 35;

  hal_lcd_clear(COLOR_BLACK);

  /* Configure video driver for input */
  //if (VideoDrv_Configure(VIDEO_DRV_IN0,  IMAGE_WIDTH, IMAGE_HEIGHT, VIDEO_DRV_COLOR_RGB888, FRAME_RATE) != VIDEO_DRV_OK) {
  if (VideoDrv_Configure(VIDEO_DRV_IN0,  IMAGE_WIDTH, IMAGE_HEIGHT, VIDEO_DRV_COLOR_GRAYSCALE8, FRAME_RATE) != VIDEO_DRV_OK) {
    log_error("Failed to configure video input\n");
    return;
  }

  if (VideoDrv_Configure(VIDEO_DRV_OUT0,  IMAGE_WIDTH, IMAGE_HEIGHT, VIDEO_DRV_COLOR_GRAYSCALE8, FRAME_RATE) != VIDEO_DRV_OK) {
    log_error("Failed to configure video input\n");
    return;
  }

  /* Set input video buffer */
  if (VideoDrv_SetBuf(VIDEO_DRV_IN0,  ImageBuf, IMAGE_WIDTH*IMAGE_HEIGHT/*CHANNELS_IMAGE_DISPLAYED*/) != VIDEO_DRV_OK) {
    log_error("Failed to set buffer for video input\n");
    return;
  }
  q15_t* Buffer_tmp = (q15_t*)malloc(arm_get_scratch_size_generic_15(IMAGE_WIDTH));
  //q15_t* Buffer_tmp = (q15_t*)malloc(arm_cv_get_scratch_size_canny_sobel(IMAGE_WIDTH));
  //q15_t* Buffer_tmp = (q15_t*)(&ImageBuf[0] + 192*192 * 2);
  if(Buffer_tmp==NULL)
  {
    printf("issue1\n");
  }
  int border_type = ARM_CV_BORDER_NEAREST;
  uint8_t* imgBuffR = &ImageBuf[0] + 192*192*sizeof(uint8_t);

  arm_cv_image_gray8_t output={(uint16_t)IMAGE_WIDTH,(uint16_t)IMAGE_HEIGHT,(uint8_t*)ImageBufOut};  
  
  if (VideoDrv_SetBuf(VIDEO_DRV_OUT0,  ImageBufOut, IMAGE_WIDTH*IMAGE_HEIGHT) != VIDEO_DRV_OK) {
    log_error("Failed to set buffer for video output\n");
    return;
  }

  /* Set input file */
  if (VideoDrv_SetFile(VIDEO_DRV_IN0, INPUT_IMAGE) != VIDEO_DRV_OK) {
    log_error("Failed to set filename for video input\n");
    return;
  }

  if (VideoDrv_SetFile(VIDEO_DRV_OUT0, OUTPUT_IMAGE) != VIDEO_DRV_OK) {
    log_error("Failed to set filename for video input\n");
    return;
  }
  
  /* Loop for obtaining video frames */
  while (1) {

    VideoDrv_Status_t status;
    /* Start video capture */
    if (VideoDrv_StreamStart(VIDEO_DRV_IN0, VIDEO_DRV_MODE_SINGLE) != VIDEO_DRV_OK) {
      log_error("Failed to start frame capture");
      return;
    }
    /* Wait for video input frame */
    do {  
      status = VideoDrv_GetStatus(VIDEO_DRV_IN0);
      if (status.overflow != 0U) {
        log_info("Overflow\n");
      }
    } while (status.buf_empty != 0U);

    /* Stop video stream upon end of stream status */
    if (status.eos != 0U) {
      VideoDrv_StreamStop(VIDEO_DRV_IN0);
    }

    /* Get input video frame buffer */
    imgFrame = VideoDrv_GetFrameBuf(VIDEO_DRV_IN0);
    arm_cv_image_gray8_t input={(uint16_t)IMAGE_WIDTH,(uint16_t)IMAGE_HEIGHT,(uint8_t*)imgFrame};
    if (imgFrame == NULL ) {
      log_error("Invalid frame.\n");
      break;
    }
    //arm_cv_canny_edge_sobel(&input,&output, Buffer_tmp, 78,33);
    arm_gaussian_filter_5x5_fixp(&input,&output, Buffer_tmp, border_type);   

    VideoDrv_ReleaseFrame(VIDEO_DRV_IN0);    
    
    hal_lcd_display_image(
      ImageBufOut,
      IMAGE_HEIGHT,
      IMAGE_WIDTH,
      1,
      0,
      0,
      dataPsnImgDownscaleFactor);

    /* Release input frame */
    outFrame = VideoDrv_GetFrameBuf(VIDEO_DRV_OUT0);
    memcpy(outFrame, ImageBufOut, IMAGE_WIDTH*IMAGE_HEIGHT);
    VideoDrv_ReleaseFrame(VIDEO_DRV_OUT0);
    if (VideoDrv_StreamStart(VIDEO_DRV_OUT0, VIDEO_DRV_MODE_SINGLE) != VIDEO_DRV_OK) {
    log_error("Failed to start frame capture");
    return;
    }
    if (status.eos != 0U) {
            while (VideoDrv_GetStatus(VIDEO_DRV_OUT0).buf_empty == 0U);
            break;
    }
  }
  free(Buffer_tmp);
  log_info("Video Stream stopped");

  /* De-initialize video interface */
  VideoDrv_Uninitialize();

  return;
}

/*---------------------------------------------------------------------------
 * User application main thread
 *---------------------------------------------------------------------------*/
__NO_RETURN void app_main(void *argument)
{
  (void)argument;

  app_init(); // Initialization step
  app_run();  // Application run step

  log_info("Application run ended");

  for (;;){;}
}