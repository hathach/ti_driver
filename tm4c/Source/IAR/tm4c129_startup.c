//*****************************************************************************
//
// startup_gcc.c - Startup code for use with GNU tools.
//
// Copyright (c) 2013-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.2.0.295 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <system_TM4C129.h>
#include <TM4C129.h>

#define SRAM_START 0x20000000
#define SRAM_SIZE  (256*1024)
#define SRAM_END   (SRAM_START + SRAM_SIZE)

//*****************************************************************************
//
//  Bit definitions for the FPU CPAC register.
//
//*****************************************************************************
#define CPAC_CP11_M        0x00C00000  // CP11 Coprocessor Access
                                            // Privilege
#define CPAC_CP11_DIS      0x00000000  // Access Denied
#define CPAC_CP11_PRIV     0x00400000  // Privileged Access Only
#define CPAC_CP11_FULL     0x00C00000  // Full Access
#define CPAC_CP10_M        0x00300000  // CP10 Coprocessor Access
                                            // Privilege
#define CPAC_CP10_DIS      0x00000000  // Access Denied
#define CPAC_CP10_PRIV     0x00100000  // Privileged Access Only
#define CPAC_CP10_FULL     0x00300000  // Full Access

#define init_MSP           SRAM_END
//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void Reset_Handler(void);
static void Default_Handler(void);

void NMI_Handler 					(void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler 				(void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler 				(void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler 				(void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler 			(void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler 					(void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler 				(void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler   				(void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler  				(void) __attribute__ ((weak, alias("Default_Handler")));

//*****************************************************************************
//
// External declarations for the interrupt handlers used by the application.
//
//*****************************************************************************
extern void SysTick_Handler(void);
extern void USB0_Handler(void);

//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int main(void);
extern void __iar_program_start(void);

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
#pragma segment="CSTACK"

#pragma section = ".intvec"
#pragma location = ".intvec"
__root const uintptr_t __vector_table[] =
{
    (uintptr_t)__sfe("CSTACK"),               // The initial stack pointer
    (uintptr_t)Reset_Handler,                          // The reset handler
    (uintptr_t)NMI_Handler,                            // The NMI handler
    (uintptr_t)HardFault_Handler,                      // The hard fault handler
    (uintptr_t)MemManage_Handler,                      // The MPU fault handler
    (uintptr_t)BusFault_Handler,                       // The bus fault handler
    (uintptr_t)UsageFault_Handler,                     // The usage fault handler
    (uintptr_t)0,                                      // Reserved
    (uintptr_t)0,                                      // Reserved
    (uintptr_t)0,                                      // Reserved
    (uintptr_t)0,                                      // Reserved
    (uintptr_t)SVC_Handler,                            // SVCall handler
    (uintptr_t)DebugMon_Handler,                       // Debug monitor handler
    (uintptr_t)0,                                      // Reserved
    (uintptr_t)PendSV_Handler,                         // The PendSV handler
    (uintptr_t)SysTick_Handler,                        // The SysTick handler
    (uintptr_t)Default_Handler,                        // GPIO Port A
    (uintptr_t)Default_Handler,                        // GPIO Port B
    (uintptr_t)Default_Handler,                        // GPIO Port C
    (uintptr_t)Default_Handler,                        // GPIO Port D
    (uintptr_t)Default_Handler,                        // GPIO Port E
    (uintptr_t)Default_Handler,                        // UART0 Rx and Tx
    (uintptr_t)Default_Handler,                        // UART1 Rx and Tx
    (uintptr_t)Default_Handler,                        // SSI0 Rx and Tx
    (uintptr_t)Default_Handler,                        // I2C0 Master and Slave
    (uintptr_t)Default_Handler,                        // PWM Fault
    (uintptr_t)Default_Handler,                        // PWM Generator 0
    (uintptr_t)Default_Handler,                        // PWM Generator 1
    (uintptr_t)Default_Handler,                        // PWM Generator 2
    (uintptr_t)Default_Handler,                        // Quadrature Encoder 0
    (uintptr_t)Default_Handler,                        // ADC Sequence 0
    (uintptr_t)Default_Handler,                        // ADC Sequence 1
    (uintptr_t)Default_Handler,                        // ADC Sequence 2
    (uintptr_t)Default_Handler,                        // ADC Sequence 3
    (uintptr_t)Default_Handler,                        // Watchdog timer
    (uintptr_t)Default_Handler,                        // Timer 0 subtimer A
    (uintptr_t)Default_Handler,                        // Timer 0 subtimer B
    (uintptr_t)Default_Handler,                        // Timer 1 subtimer A
    (uintptr_t)Default_Handler,                        // Timer 1 subtimer B
    (uintptr_t)Default_Handler,                        // Timer 2 subtimer A
    (uintptr_t)Default_Handler,                        // Timer 2 subtimer B
    (uintptr_t)Default_Handler,                        // Analog Comparator 0
    (uintptr_t)Default_Handler,                        // Analog Comparator 1
    (uintptr_t)Default_Handler,                        // Analog Comparator 2
    (uintptr_t)Default_Handler,                        // System Control (PLL, OSC, BO)
    (uintptr_t)Default_Handler,                        // FLASH Control
    (uintptr_t)Default_Handler,                        // GPIO Port F
    (uintptr_t)Default_Handler,                        // GPIO Port G
    (uintptr_t)Default_Handler,                        // GPIO Port H
    (uintptr_t)Default_Handler,                        // UART2 Rx and Tx
    (uintptr_t)Default_Handler,                        // SSI1 Rx and Tx
    (uintptr_t)Default_Handler,                        // Timer 3 subtimer A
    (uintptr_t)Default_Handler,                        // Timer 3 subtimer B
    (uintptr_t)Default_Handler,                        // I2C1 Master and Slave
    (uintptr_t)Default_Handler,                        // CAN0
    (uintptr_t)Default_Handler,                        // CAN1
    (uintptr_t)Default_Handler,                        // Ethernet
    (uintptr_t)Default_Handler,                        // Hibernate
    (uintptr_t)USB0_Handler,                           // USB0
    (uintptr_t)Default_Handler,                        // PWM Generator 3
    (uintptr_t)Default_Handler,                        // uDMA Software Transfer
    (uintptr_t)Default_Handler,                        // uDMA Error
    (uintptr_t)Default_Handler,                        // ADC1 Sequence 0
    (uintptr_t)Default_Handler,                        // ADC1 Sequence 1
    (uintptr_t)Default_Handler,                        // ADC1 Sequence 2
    (uintptr_t)Default_Handler,                        // ADC1 Sequence 3
    (uintptr_t)Default_Handler,                        // External Bus Interface 0
    (uintptr_t)Default_Handler,                        // GPIO Port J
    (uintptr_t)Default_Handler,                        // GPIO Port K
    (uintptr_t)Default_Handler,                        // GPIO Port L
    (uintptr_t)Default_Handler,                        // SSI2 Rx and Tx
    (uintptr_t)Default_Handler,                        // SSI3 Rx and Tx
    (uintptr_t)Default_Handler,                        // UART3 Rx and Tx
    (uintptr_t)Default_Handler,                        // UART4 Rx and Tx
    (uintptr_t)Default_Handler,                        // UART5 Rx and Tx
    (uintptr_t)Default_Handler,                        // UART6 Rx and Tx
    (uintptr_t)Default_Handler,                        // UART7 Rx and Tx
    (uintptr_t)Default_Handler,                        // I2C2 Master and Slave
    (uintptr_t)Default_Handler,                        // I2C3 Master and Slave
    (uintptr_t)Default_Handler,                        // Timer 4 subtimer A
    (uintptr_t)Default_Handler,                        // Timer 4 subtimer B
    (uintptr_t)Default_Handler,                        // Timer 5 subtimer A
    (uintptr_t)Default_Handler,                        // Timer 5 subtimer B
    (uintptr_t)Default_Handler,                        // FPU
    (uintptr_t)0,                                      // Reserved
    (uintptr_t)0,                                      // Reserved
    (uintptr_t)Default_Handler,                        // I2C4 Master and Slave
    (uintptr_t)Default_Handler,                        // I2C5 Master and Slave
    (uintptr_t)Default_Handler,                        // GPIO Port M
    (uintptr_t)Default_Handler,                        // GPIO Port N
    (uintptr_t)0,                                      // Reserved
    (uintptr_t)Default_Handler,                        // Tamper
    (uintptr_t)Default_Handler,                        // GPIO Port P (Summary or P0)
    (uintptr_t)Default_Handler,                        // GPIO Port P1
    (uintptr_t)Default_Handler,                        // GPIO Port P2
    (uintptr_t)Default_Handler,                        // GPIO Port P3
    (uintptr_t)Default_Handler,                        // GPIO Port P4
    (uintptr_t)Default_Handler,                        // GPIO Port P5
    (uintptr_t)Default_Handler,                        // GPIO Port P6
    (uintptr_t)Default_Handler,                        // GPIO Port P7
    (uintptr_t)Default_Handler,                        // GPIO Port Q (Summary or Q0)
    (uintptr_t)Default_Handler,                        // GPIO Port Q1
    (uintptr_t)Default_Handler,                        // GPIO Port Q2
    (uintptr_t)Default_Handler,                        // GPIO Port Q3
    (uintptr_t)Default_Handler,                        // GPIO Port Q4
    (uintptr_t)Default_Handler,                        // GPIO Port Q5
    (uintptr_t)Default_Handler,                        // GPIO Port Q6
    (uintptr_t)Default_Handler,                        // GPIO Port Q7
    (uintptr_t)Default_Handler,                        // GPIO Port R
    (uintptr_t)Default_Handler,                        // GPIO Port S
    (uintptr_t)Default_Handler,                        // SHA/MD5 0
    (uintptr_t)Default_Handler,                        // AES 0
    (uintptr_t)Default_Handler,                        // DES3DES 0
    (uintptr_t)Default_Handler,                        // LCD Controller 0
    (uintptr_t)Default_Handler,                        // Timer 6 subtimer A
    (uintptr_t)Default_Handler,                        // Timer 6 subtimer B
    (uintptr_t)Default_Handler,                        // Timer 7 subtimer A
    (uintptr_t)Default_Handler,                        // Timer 7 subtimer B
    (uintptr_t)Default_Handler,                        // I2C6 Master and Slave
    (uintptr_t)Default_Handler,                        // I2C7 Master and Slave
    (uintptr_t)Default_Handler,                        // HIM Scan Matrix Keyboard 0
    (uintptr_t)Default_Handler,                        // One Wire 0
    (uintptr_t)Default_Handler,                        // HIM PS/2 0
    (uintptr_t)Default_Handler,                        // HIM LED Sequencer 0
    (uintptr_t)Default_Handler,                        // HIM Consumer IR 0
    (uintptr_t)Default_Handler,                        // I2C8 Master and Slave
    (uintptr_t)Default_Handler,                        // I2C9 Master and Slave
    (uintptr_t)Default_Handler                         // GPIO Port T
};

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void
Reset_Handler(void)
{
    // Enable the floating-point unit.  This must be done here to handle the
    // case where main() uses floating-point and the function prologue saves
    // floating-point registers (which will fault if floating-point is not
    // enabled).
    SCB->CPACR &= ~ (CPAC_CP10_M | CPAC_CP11_M);
    SCB->CPACR |= (CPAC_CP10_FULL | CPAC_CP11_FULL);

    //__libc_init_array();
    SystemInit();

    //
    // Call the application's entry point.
    //
    __iar_program_start();
}


//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
Default_Handler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}
