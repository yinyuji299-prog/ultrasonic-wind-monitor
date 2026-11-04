////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013-2025 Efinix Inc. All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "soc.h"
#include "uart.h"
#include "clint.h"
#include "semihosting.h"

////////////////////////////////////////////////////////////////////////////////
    /*
    *   Default main peripherals for SoC.
    */
////////////////////////////////////////////////////////////////////////////////

#define BSP_PLIC            SYSTEM_PLIC_CTRL
#define BSP_PLIC_CPU_0      SYSTEM_PLIC_SYSTEM_CORES_0_EXTERNAL_INTERRUPT
#define BSP_UART_TERMINAL   SYSTEM_UART_0_IO_CTRL
/* 2026-10-27：100 MHz / (961538 x 8) = 13.0000 -> 分频写 12，无截断误差。
 * 原先写 921600 时 100e6/(921600*8)=13.56 会被截断成 13，实际仍是 961538，
 * 但宏值对不上实际波特率，上位机按 921600 收会因 +4.3% 偏差而乱码。
 * 这里直接写实际值，避免再算错。 */
#define BSP_UART_BAUDRATE   961538
#define BSP_UART_DATA_LEN   8
#define bsp_putChar(c)      uart_write(BSP_UART_TERMINAL, c);
#define BSP_CLINT           SYSTEM_CLINT_CTRL
#define BSP_CLINT_HZ        SYSTEM_CLINT_HZ
#define bsp_uDelay(usec)    clint_uDelay(usec, SYSTEM_CLINT_HZ, SYSTEM_CLINT_CTRL);

#ifdef SYSTEM_PLIC_SYSTEM_CORES_1_EXTERNAL_INTERRUPT
#define BSP_PLIC_CPU_1 SYSTEM_PLIC_SYSTEM_CORES_1_EXTERNAL_INTERRUPT
#endif

#ifdef SYSTEM_PLIC_SYSTEM_CORES_2_EXTERNAL_INTERRUPT
#define BSP_PLIC_CPU_2 SYSTEM_PLIC_SYSTEM_CORES_2_EXTERNAL_INTERRUPT
#endif

#ifdef SYSTEM_PLIC_SYSTEM_CORES_3_EXTERNAL_INTERRUPT
#define BSP_PLIC_CPU_3 SYSTEM_PLIC_SYSTEM_CORES_3_EXTERNAL_INTERRUPT
#endif

////////////////////////////////////////////////////////////////////////////////
    /*
    *   Support printing for char, string , decimal and hexadecinaml specifier.
    *   Support print_float and print_dec function.
    *   Uses moderate memory resource.
    */
////////////////////////////////////////////////////////////////////////////////
#define ENABLE_BSP_PRINTF                   1

////////////////////////////////////////////////////////////////////////////////
    /*
    *   Full support for printf function, including flag and precisions.
    *   Uses more memory resource.
    *
    *   This is the "library functions" switch the RISC-V IDE exposes under
    *   Project Properties -> BSP / print.  Leave it ON so %u/%x width flags
    *   and the full specifier set resolve in the indexer (otherwise CDT
    *   reports unresolved names / syntax errors on every formatted print).
    */
////////////////////////////////////////////////////////////////////////////////
#define ENABLE_BSP_PRINTF_FULL              1

////////////////////////////////////////////////////////////////////////////////
    /*
    *   Enable the function sharing between 'full' and 'lite' printing.
    */
////////////////////////////////////////////////////////////////////////////////
#define ENABLE_BRIDGE_FULL_TO_LITE          1

////////////////////////////////////////////////////////////////////////////////
    /*
    *   Enable more printing specifier like floating point, exponential,
    *   pointer difference and long long data type.
    */
////////////////////////////////////////////////////////////////////////////////
#define ENABLE_FLOATING_POINT_SUPPORT       SYSTEM_CORES_0_FPU
#define ENABLE_FP_EXPONENTIAL_SUPPORT       0
#define ENABLE_PTRDIFF_SUPPORT              0                                                                        
#define ENABLE_LONG_LONG_SUPPORT            0

////////////////////////////////////////////////////////////////////////////////
    /*
    *   Warning message when specifier is not supported.
    */
////////////////////////////////////////////////////////////////////////////////
#define ENABLE_PRINTF_WARNING               1

////////////////////////////////////////////////////////////////////////////////
    /*
    *   Enable printing with semihosting.
    */
////////////////////////////////////////////////////////////////////////////////
#define ENABLE_SEMIHOSTING_PRINT            0


#if (ENABLE_BSP_PRINTF)
    #include "print.h"
#endif //#if (ENABLE_BSP_PRINTF)

#if (ENABLE_BSP_PRINTF_FULL)
    #if (!ENABLE_FLOATING_POINT_SUPPORT)
        #define PRINTF_DISABLE_SUPPORT_FLOAT 1
    #endif //#if (ENABLE_FLOATING_POINT_SUPPORT)
    
    #if (!ENABLE_FP_EXPONENTIAL_SUPPORT)
        #define PRINTF_DISABLE_SUPPORT_EXPONENTIAL 1
    #endif //#if (ENABLE_FP_EXPONENTIAL_SUPPORT)
    
    #if (!ENABLE_PTRDIFF_SUPPORT)
        #define PRINTF_DISABLE_SUPPORT_PTRDIFF_T 1
    #endif //#if (ENABLE_PTRDIFF_SUPPORT)
    
    #if (!ENABLE_LONG_LONG_SUPPORT)
        #define PRINTF_DISABLE_SUPPORT_LONG_LONG 1
    #endif //#if (ENABLE_LONG_LONG_SUPPORT)
    
    #if(ENABLE_BRIDGE_FULL_TO_LITE)
        #if (!ENABLE_BSP_PRINTF)
            #define bsp_printf bsp_printf_full
        #endif // #if (!ENABLE_BSP_PRINTF)
    #endif //#if(ENABLE_BRIDGE_EFX_TO_BSP)
    #include "print_full.h"
    
#endif //#if (ENABLE_BSP_PRINTF_FULL)


////////////////////////////////////////////////////////////////////////////////
    /*
    *   Peripherals initialization routine when power up the SoC.
    *   1. UART baudrate
    *   2. 
    */
////////////////////////////////////////////////////////////////////////////////
    static void bsp_init()
    {
        Uart_Config uartConfig;
        uartConfig.dataLength   = BITS_8;
        uartConfig.parity       = NONE;
        uartConfig.stop         = ONE;
        uartConfig.clockDivider = BSP_CLINT_HZ/(BSP_UART_BAUDRATE*BSP_UART_DATA_LEN)-1;
        uart_applyConfig(BSP_UART_TERMINAL, &uartConfig);    
    }
