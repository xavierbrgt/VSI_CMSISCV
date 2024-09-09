# CMSIS Build CMakeLists generated on 2024-09-09T14:54:48

cmake_minimum_required(VERSION 3.22)

# Target options

set(TARGET hello_video_vsi)
set(CPU Cortex-M55)
set(PRJ_DIR "C:/Users/xavbou01/AVH-VSI/hello_video_vsi")
set(OUT_DIR "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/out/hello_video_vsi/Corstone_300/lcd_out/AC6")
set(INT_DIR "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/tmp/hello_video_vsi/Corstone_300/lcd_out")
set(FPU DP_FPU)
set(DSP DSP)
set(TZ TZ)
set(SECURE TZ-disabled)
set(MVE FP_MVE)
set(DEBUG on)
set(AS_FLAGS_GLOBAL "-masm=auto")
set(CC_FLAGS_GLOBAL "-std=gnu11 -Wno-license-management -Wno-documentation -Wno-documentation-unknown-command -Wno-missing-noreturn -Wno-missing-prototypes -Wno-missing-variable-declarations -Wno-nonportable-include-path -Wno-packed -Wno-parentheses-equality -Wno-reserved-id-macro -Wno-reserved-identifier -Wno-unused-macros -fno-exceptions -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -Wno-sign-conversion")
set(CXX_FLAGS_GLOBAL "-xc++ -std=c++14 -Wno-documentation -Wno-documentation-unknown-command -Wno-license-management -Wno-missing-noreturn -Wno-missing-prototypes -Wno-missing-variable-declarations -Wno-nonportable-include-path -Wno-packed -Wno-parentheses-equality -Wno-reserved-id-macro -Wno-reserved-identifier -Wno-unused-macros -fno-exceptions -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -Wno-sign-conversion")
set(LD_FLAGS_GLOBAL "--map --load_addr_map_info --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --summary_stderr --info summarysizes --entry=Reset_Handler --strict --diag_suppress=6314,6848")
set(LD_SCRIPT "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/RTE/Device/SSE-300-MPS3/fvp_sse300_mps3_s.sct")

set(DEFINES
  _RTE_
  CORSTONE300_FVP
)

set(INC_PATHS
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/application"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/micro_logger"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/vsi"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/vsi/video_driver"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/include"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/components/lcd/include"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/components/lcd/source"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/components/stdout/include"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/platform_pmu/include"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Include"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/PrivateInclude"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/platform/mps3/include/sse-300"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/platform/mps3/include"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/platform/mps3/source"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/RTE/CMSIS"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/RTE/Device/SSE-300-MPS3"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/RTE/_lcd_out_Corstone_300"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-Compiler/2.1.0/include"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Include"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/PrivateInclude"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Include"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS/6.1.0/CMSIS/Core/Include"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS/6.1.0/CMSIS/Driver/Include"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS/6.1.0/CMSIS/RTOS2/Include"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Board/Device_Definition"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Board/Platform"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/CMSIS_Driver"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/CMSIS_Driver/Config"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Device/Include"
)

set(ASM_SRC_FILES
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/GCC/irq_armv8mml.S"
)

set(CC_SRC_FILES
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/RTE/CMSIS/RTX_Config.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/RTE/Device/SSE-300-MPS3/retarget.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/RTE/Device/SSE-300-MPS3/startup_SSE300MPS3.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/RTE/Device/SSE-300-MPS3/system_SSE300MPS3.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/application/app.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/application/main.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/components/lcd/source/glcd_mps3/glcd_mps3.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/components/lcd/source/lcd_img.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/components/stdout/source/user_input.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/hal.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/hal_pmu.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/platform/mps3/source/platform_drivers.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/hal/source/platform/mps3/source/timer_mps3.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/micro_logger/micro_logger.c"
  "C:/Users/xavbou01/AVH-VSI/hello_video_vsi/source/vsi/video_driver/video_drv.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-Compiler/2.1.0/source/armcc/retarget_io.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/BasicMathFunctions/BasicMathFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/BasicMathFunctions/BasicMathFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/BayesFunctions/BayesFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/BayesFunctions/BayesFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/CommonTables/CommonTables.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/CommonTables/CommonTablesF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/ComplexMathFunctions/ComplexMathFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/ComplexMathFunctions/ComplexMathFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/ControllerFunctions/ControllerFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/DistanceFunctions/DistanceFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/DistanceFunctions/DistanceFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/FastMathFunctions/FastMathFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/FastMathFunctions/FastMathFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/FilteringFunctions/FilteringFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/FilteringFunctions/FilteringFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/InterpolationFunctions/InterpolationFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/InterpolationFunctions/InterpolationFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/MatrixFunctions/MatrixFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/MatrixFunctions/MatrixFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/QuaternionMathFunctions/QuaternionMathFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/SVMFunctions/SVMFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/SVMFunctions/SVMFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/StatisticsFunctions/StatisticsFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/StatisticsFunctions/StatisticsFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/SupportFunctions/SupportFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/SupportFunctions/SupportFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/TransformFunctions/TransformFunctions.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Source/TransformFunctions/TransformFunctionsF16.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_delay.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_evflags.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_evr.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_kernel.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_lib.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_memory.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_mempool.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_msgqueue.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_mutex.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_semaphore.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_system.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_thread.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS-RTX/5.9.0/Source/rtx_timer.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/CMSIS/6.1.0/CMSIS/RTOS2/Source/os_systick.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Board/Device_Definition/device_definition.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/CMSIS_Driver/Driver_USART.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Device/Source/Driver_SSE300_PPC.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Device/Source/arm_mps3_io_drv.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Device/Source/ppc_sse300_drv.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Device/Source/syscounter_armv8-m_cntrl_drv.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Device/Source/syscounter_armv8-m_read_drv.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Device/Source/systimer_armv8-m_drv.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Device/Source/systimer_armv8-m_timeout.c"
  "C:/Users/xavbou01/AppData/Local/Arm/Packs/ARM/V2M_MPS3_SSE_300_BSP/1.5.0/Device/Source/uart_cmsdk_drv.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ColorTransforms/arm_bgr_8U3C_to_gray8.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ColorTransforms/arm_bgr_8U3C_to_rgb24.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ColorTransforms/arm_gray8_to_rgb24.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ColorTransforms/arm_rgb24_to_gray8.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ColorTransforms/arm_yuv420_to_gray8.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ColorTransforms/arm_yuv420_to_rgb24.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/FeatureDetection/arm_cannysobel.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ImageTransforms/arm_crop_gray8.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ImageTransforms/arm_crop_rgb24.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ImageTransforms/arm_image_resize_bgr_8U3C_f32.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ImageTransforms/arm_image_resize_bgr_8U3C_to_rgb24_f32.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ImageTransforms/arm_image_resize_common_f32.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/ImageTransforms/arm_image_resize_gray8_f32.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/LinearFilters/arm_gaussian.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/LinearFilters/arm_gaussian_5x5.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/LinearFilters/arm_gaussian_7x7.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/LinearFilters/arm_gaussian_7x7_32.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/LinearFilters/arm_linear_common.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/LinearFilters/arm_sobel_horizontal.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/LinearFilters/arm_sobel_vertical.c"
  "C:/Users/xavbou01/CMSIS-CV/CMSIS-CV/Source/arm_cv_common.c"
)

# Toolchain config map

set(REGISTERED_TOOLCHAIN_ROOT "C:/Keil_v5/ARM/ARMCLANG/bin")
set(REGISTERED_TOOLCHAIN_VERSION "6.21.0")
set(TOOLCHAIN_VERSION_MIN "6.21.0")
include ("C:/ctools/cmsis-toolbox-windows-amd64/etc/AC6.6.21.0.cmake")
include ("C:/ctools/cmsis-toolbox-windows-amd64/etc/CMSIS-Build-Utils.cmake")

# Setup project

project(${TARGET} LANGUAGES ASM C)

cbuild_get_running_toolchain(TOOLCHAIN_ROOT TOOLCHAIN_VERSION C)

# Global Flags

set(CMAKE_ASM_FLAGS "${ASM_CPU} ${ASM_DEFINES} ${ASM_OPTIONS_FLAGS} ${ASM_FLAGS} ${AS_FLAGS_GLOBAL}")
cbuild_get_system_includes(CC_SYS_INC_PATHS_LIST CC_SYS_INC_PATHS)
set(CMAKE_C_FLAGS "${CC_CPU} ${CC_DEFINES} ${CC_SECURE} ${CC_OPTIONS_FLAGS} ${CC_FLAGS} ${CC_FLAGS_GLOBAL} ${CC_SYS_INC_PATHS}")
set(CMAKE_C_LINK_FLAGS "${LD_CPU} ${_LS}\"${LD_SCRIPT}\" ${LD_SECURE} ${LD_FLAGS_GLOBAL} ${LD_OPTIONS_FLAGS} ${LD_FLAGS}")

# Compilation Database

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_custom_target(database COMMAND ${CMAKE_COMMAND} -E copy_if_different "${INT_DIR}/compile_commands.json" "${OUT_DIR}")

# Setup Target

add_executable(${TARGET} ${ASM_SRC_FILES} ${CC_SRC_FILES})
set_target_properties(${TARGET} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "hello_video_vsi")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR} LINK_DEPENDS ${LD_SCRIPT})
target_include_directories(${TARGET} PUBLIC ${INC_PATHS})
