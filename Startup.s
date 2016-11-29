; <<< Use Configuration Wizard in Context Menu >>>
;******************************************************************************
;
; Startup.s - Startup code for Stellaris.
;
; Copyright (c) 2011 Texas Instruments Incorporated.  All rights reserved.
; Software License Agreement
; 
; Texas Instruments (TI) is supplying this software for use solely and
; exclusively on TI's microcontroller products. The software is owned by
; TI and/or its suppliers, and is protected under applicable copyright
; laws. You may not combine this software with "viral" open-source
; software in order to form a larger program.
; 
; THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
; NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
; NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
; CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
; DAMAGES, FOR ANY REASON WHATSOEVER.
; 
; This is part of revision 7860 of the Stellaris Peripheral Driver Library.
;
;******************************************************************************

;******************************************************************************
;
; <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;
;******************************************************************************
Stack   EQU     0x00000100

;******************************************************************************
;
; <o> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
;
;******************************************************************************
Heap    EQU     0x00000000

;******************************************************************************
;
; Allocate space for the stack.
;
;******************************************************************************
        AREA    STACK, NOINIT, READWRITE, ALIGN=3
StackMem
        SPACE   Stack
__initial_sp

;******************************************************************************
;
; Allocate space for the heap.
;
;******************************************************************************
        AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
HeapMem
        SPACE   Heap
__heap_limit

;******************************************************************************
;
; Indicate that the code in this file preserves 8-byte alignment of the stack.
;
;******************************************************************************
        PRESERVE8

;******************************************************************************
;
; Place code into the reset code section.
;
;******************************************************************************
        AREA    RESET, CODE, READONLY
		THUMB
		EXTERN            SysTick_Handler    
		EXTERN            GPIOA_Handler      
		EXTERN            GPIOB_Handler      
		EXTERN            GPIOC_Handler      
		EXTERN            GPIOD_Handler      
		EXTERN            GPIOE_Handler      
		EXTERN            UART0_Handler      
		EXTERN            UART1_Handler      
		EXTERN            SSI0_Handler       
		EXTERN            I2C0_Handler       
		EXTERN            PWM0_Fault_Handler 
		EXTERN            PWM0_G0_Handler    
		EXTERN            PWM0_G1_Handler    
		EXTERN            PWM0_G2_Handler    
		EXTERN            QE_Handler         
		EXTERN            ADC0_S0_Handler    
		EXTERN            ADC0_S1_Handler    
		EXTERN            ADC0_S2_Handler    
		EXTERN            ADC0_S3_Handler    
		EXTERN            WDOG_Timer_Handler 
		EXTERN            TIMER0_STA_Handler 
		EXTERN            TIMER0_STB_Handler 
		EXTERN            TIMER1_STA_Handler 
		EXTERN            TIMER1_STB_Handler 
		EXTERN            TIMER2_STA_Handler 
		EXTERN            TIMER2_STB_Handler 
		EXTERN            COMP0_Handler      
		EXTERN            COMP1_Handler      
		EXTERN            COMP2_Handler      
		EXTERN            SYSCTL_Handler     
		EXTERN            FLASH_Handler      
		EXTERN            GPIOF_Handler      
		EXTERN            GPIOG_Handler      
		EXTERN            GPIOH_Handler      
		EXTERN            UART2_Handler      
		EXTERN            SSI1_Handler       
		EXTERN            TIMER3_STA_Handler 
		EXTERN            TIMER3_STB_Handler 
		EXTERN            I2C1_Handler       
		EXTERN            QE1_Handler        
		EXTERN            CAN0_Handler       
		EXTERN            CA1_Handler        
		EXTERN            CA2_Handler        
		EXTERN            Ethernet_Handler   
		EXTERN            Hibernate_Handler  
		EXTERN            USB0_Handler       
		EXTERN            PWM_G3_Handler     
		EXTERN            UDMA_SUCCESS_Handler
		EXTERN            UDMA_ERROR_Handler 
		EXTERN            ADC1_S0_Handler    
		EXTERN            ADC1_S1_Handler    
		EXTERN            ADC1_S2_Handler    
		EXTERN            ADC1_S3_Handler    
		EXTERN            I2S0_Handler       
		EXTERN            EBI0_Handler       
		EXTERN            GPIOJ_Handler      
		EXTERN            GPIOK_Handler      
		EXTERN            GPIOL_Handler      
		EXTERN            SSI2_Handler       
		EXTERN            SSI3_Handler       
		EXTERN            UART3_Handler      
		EXTERN            UART4_Handler      
		EXTERN            UART5_Handler      
		EXTERN            UART6_Handler      
		EXTERN            UART7_Handler                      
		EXTERN            I2C2_Handler       
		EXTERN            I2C3_Handler       
		EXTERN            TIMER4_STA_Handler 
		EXTERN            TIMER4_STB_Handler              
		EXTERN            TIMER5_STA_Handler 
		EXTERN            TIMER5_STB_Handler 
		EXTERN            WTIMER0_STA_Handler
		EXTERN            WTIMER0_STB_Handler
		EXTERN            WTIMER1_STA_Handler
		EXTERN            WTIMER1_STB_Handler
		EXTERN            WTIMER2_STA_Handler
		EXTERN            WTIMER2_STB_Handler
		EXTERN            WTIMER3_STA_Handler
		EXTERN            WTIMER3_STB_Handler
		EXTERN            WTIMER4_STA_Handler
		EXTERN            WTIMER4_STB_Handler
		EXTERN            WTIMER5_STA_Handler
		EXTERN            WTIMER5_STB_Handler
		EXTERN            FPU_Handler        
		EXTERN            PECI0_Handler      
		EXTERN            LPC0_Handler       
		EXTERN            I2C4_Handler       
		EXTERN            I2C5_Handler       
		EXTERN            GPIOM_Handler      
		EXTERN            GPION_Handler      
		EXTERN            QE2_Handler        
		EXTERN            Fan0_Handler                       
		EXTERN            GPIOP0_Handler     
		EXTERN            GPIOP1_Handler     
		EXTERN            GPIOP2_Handler     
		EXTERN            GPIOP3_Handler     
		EXTERN            GPIOP4_Handler     
		EXTERN            GPIOP5_Handler     
		EXTERN            GPIOP6_Handler     
		EXTERN            GPIOP7_Handler     
		EXTERN            GPIOQ0_Handler     
		EXTERN            GPIOQ1_Handler     
		EXTERN            GPIOQ2_Handler     
		EXTERN            GPIOQ3_Handler     
		EXTERN            GPIOQ4_Handler     
		EXTERN            GPIOQ5_Handler     
		EXTERN            GPIOQ6_Handler     
		EXTERN            GPIOQ7_Handler     
		EXTERN            GPIOR_Handler      
		EXTERN            GPIOS_Handler      
		EXTERN            PWM1_G0_Handler    
		EXTERN            PWM1_G1_Handler    
		EXTERN            PWM1_G2_Handler    
		EXTERN            PWM1_G3_Handler    
		EXTERN            PWM1_Fault_Handler 
;******************************************************************************
;
; The vector table.
;
;******************************************************************************
        EXPORT  __Vectors
__Vectors
        DCD     StackMem + Stack            ; Top of Stack
        DCD     Reset_Handler               ; Reset Handler
        DCD     NmiSR                       ; NMI Handler
        DCD     FaultISR                    ; Hard Fault Handler
        DCD     IntDefaultHandler           ; MPU Fault Handler
        DCD     IntDefaultHandler           ; Bus Fault Handler
        DCD     IntDefaultHandler           ; Usage Fault Handler
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     IntDefaultHandler           ; SVCall handler
        DCD     IntDefaultHandler           ; Debug monitor handler
        DCD     0                           ; Reserved
        DCD     IntDefaultHandler           ; PendSV Handler
        DCD     SysTick_Handler             ; SysTick Handler
        DCD     GPIOA_Handler               ; GPIO Port A
        DCD     GPIOB_Handler               ; GPIO Port B
        DCD     GPIOC_Handler               ; GPIO Port C
        DCD     GPIOD_Handler               ; GPIO Port D
        DCD     GPIOE_Handler               ; GPIO Port E
        DCD     UART0_Handler               ; UART0 Rx and Tx
        DCD     UART1_Handler               ; UART1 Rx and Tx
        DCD     SSI0_Handler                ; SSI0 Rx and Tx
        DCD     I2C0_Handler                ; I2C0 Master and Slave
        DCD     PWM0_Fault_Handler          ; PWM Fault
        DCD     PWM0_G0_Handler             ; PWM Generator 0
        DCD     PWM0_G1_Handler             ; PWM Generator 1
        DCD     PWM0_G2_Handler             ; PWM Generator 2
        DCD     QE_Handler                  ; Quadrature Encoder 0
        DCD     ADC0_S0_Handler             ; ADC Sequence 0
        DCD     ADC0_S1_Handler             ; ADC Sequence 1
        DCD     ADC0_S2_Handler             ; ADC Sequence 2
        DCD     ADC0_S3_Handler             ; ADC Sequence 3
        DCD     WDOG_Timer_Handler          ; Watchdog timer
        DCD     TIMER0_STA_Handler          ; Timer 0 subtimer A
        DCD     TIMER0_STB_Handler          ; Timer 0 subtimer B
        DCD     TIMER1_STA_Handler          ; Timer 1 subtimer A
        DCD     TIMER1_STB_Handler          ; Timer 1 subtimer B
        DCD     TIMER2_STA_Handler          ; Timer 2 subtimer A
        DCD     TIMER2_STB_Handler          ; Timer 2 subtimer B
        DCD     COMP0_Handler               ; Analog Comparator 0
        DCD     COMP1_Handler               ; Analog Comparator 1
        DCD     COMP2_Handler               ; Analog Comparator 2
        DCD     SYSCTL_Handler              ; System Control (PLL, OSC, BO)
        DCD     FLASH_Handler               ; FLASH Control
        DCD     GPIOF_Handler               ; GPIO Port F
        DCD     GPIOG_Handler               ; GPIO Port G
        DCD     GPIOH_Handler               ; GPIO Port H
        DCD     UART2_Handler               ; UART2 Rx and Tx
        DCD     SSI1_Handler                ; SSI1 Rx and Tx
        DCD     TIMER3_STA_Handler          ; Timer 3 subtimer A
        DCD     TIMER3_STB_Handler          ; Timer 3 subtimer B
        DCD     I2C1_Handler                ; I2C1 Master and Slave
        DCD     QE1_Handler                 ; Quadrature Encoder 1
        DCD     CAN0_Handler                ; CAN0
        DCD     CA1_Handler                 ; CAN1
        DCD     CA2_Handler                 ; CAN2
        DCD     Ethernet_Handler            ; Ethernet
        DCD     Hibernate_Handler           ; Hibernate
        DCD     USB0_Handler                ; USB0
        DCD     PWM_G3_Handler              ; PWM Generator 3
        DCD     UDMA_SUCCESS_Handler        ; uDMA Software Transfer
        DCD     UDMA_ERROR_Handler          ; uDMA Error
        DCD     ADC1_S0_Handler             ; ADC1 Sequence 0
        DCD     ADC1_S1_Handler             ; ADC1 Sequence 1
        DCD     ADC1_S2_Handler             ; ADC1 Sequence 2
        DCD     ADC1_S3_Handler             ; ADC1 Sequence 3
        DCD     I2S0_Handler                ; I2S0
        DCD     EBI0_Handler                ; External Bus Interface 0
        DCD     GPIOJ_Handler               ; GPIO Port J
        DCD     GPIOK_Handler               ; GPIO Port K
        DCD     GPIOL_Handler               ; GPIO Port L
        DCD     SSI2_Handler                ; SSI2 Rx and Tx
        DCD     SSI3_Handler                ; SSI3 Rx and Tx
        DCD     UART3_Handler               ; UART3 Rx and Tx
        DCD     UART4_Handler               ; UART4 Rx and Tx
        DCD     UART5_Handler               ; UART5 Rx and Tx
        DCD     UART6_Handler               ; UART6 Rx and Tx
        DCD     UART7_Handler               ; UART7 Rx and Tx
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     I2C2_Handler                ; I2C2 Master and Slave
        DCD     I2C3_Handler                ; I2C3 Master and Slave
        DCD     TIMER4_STA_Handler          ; Timer 4 subtimer A
        DCD     TIMER4_STB_Handler          ; Timer 4 subtimer B
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     TIMER5_STA_Handler          ; Timer 5 subtimer A
        DCD     TIMER5_STB_Handler          ; Timer 5 subtimer B
        DCD     WTIMER0_STA_Handler         ; Wide Timer 0 subtimer A
        DCD     WTIMER0_STB_Handler         ; Wide Timer 0 subtimer B
        DCD     WTIMER1_STA_Handler         ; Wide Timer 1 subtimer A
        DCD     WTIMER1_STB_Handler         ; Wide Timer 1 subtimer B
        DCD     WTIMER2_STA_Handler         ; Wide Timer 2 subtimer A
        DCD     WTIMER2_STB_Handler         ; Wide Timer 2 subtimer B
        DCD     WTIMER3_STA_Handler         ; Wide Timer 3 subtimer A
        DCD     WTIMER3_STB_Handler         ; Wide Timer 3 subtimer B
        DCD     WTIMER4_STA_Handler         ; Wide Timer 4 subtimer A
        DCD     WTIMER4_STB_Handler         ; Wide Timer 4 subtimer B
        DCD     WTIMER5_STA_Handler         ; Wide Timer 5 subtimer A
        DCD     WTIMER5_STB_Handler         ; Wide Timer 5 subtimer B
        DCD     FPU_Handler                 ; FPU
        DCD     PECI0_Handler               ; PECI 0
        DCD     LPC0_Handler                ; LPC 0
        DCD     I2C4_Handler                ; I2C4 Master and Slave
        DCD     I2C5_Handler                ; I2C5 Master and Slave
        DCD     GPIOM_Handler               ; GPIO Port M
        DCD     GPION_Handler               ; GPIO Port N
        DCD     QE2_Handler                 ; Quadrature Encoder 2
        DCD     Fan0_Handler                ; Fan 0
        DCD     0                           ; Reserved
        DCD     GPIOP0_Handler              ; GPIO Port P (Summary or P0)
        DCD     GPIOP1_Handler              ; GPIO Port P1
        DCD     GPIOP2_Handler              ; GPIO Port P2
        DCD     GPIOP3_Handler              ; GPIO Port P3
        DCD     GPIOP4_Handler              ; GPIO Port P4
        DCD     GPIOP5_Handler              ; GPIO Port P5
        DCD     GPIOP6_Handler              ; GPIO Port P6
        DCD     GPIOP7_Handler              ; GPIO Port P7
        DCD     GPIOQ0_Handler              ; GPIO Port Q (Summary or Q0)
        DCD     GPIOQ1_Handler              ; GPIO Port Q1
        DCD     GPIOQ2_Handler              ; GPIO Port Q2
        DCD     GPIOQ3_Handler              ; GPIO Port Q3
        DCD     GPIOQ4_Handler              ; GPIO Port Q4
        DCD     GPIOQ5_Handler              ; GPIO Port Q5
        DCD     GPIOQ6_Handler              ; GPIO Port Q6
        DCD     GPIOQ7_Handler              ; GPIO Port Q7
        DCD     GPIOR_Handler               ; GPIO Port R
        DCD     GPIOS_Handler               ; GPIO Port S
        DCD     PWM1_G0_Handler             ; PWM 1 Generator 0
        DCD     PWM1_G1_Handler             ; PWM 1 Generator 1
        DCD     PWM1_G2_Handler             ; PWM 1 Generator 2
        DCD     PWM1_G3_Handler             ; PWM 1 Generator 3
        DCD     PWM1_Fault_Handler          ; PWM 1 Fault

;******************************************************************************
;
; This is the code that gets called when the processor first starts execution
; following a reset event.
;
;******************************************************************************
        EXPORT  Reset_Handler
Reset_Handler
        ;
        ; Call the C library enty point that handles startup.  This will copy
        ; the .data section initializers from flash to SRAM and zero fill the
        ; .bss section.
        ;
        IMPORT  __main

        IF      {CPU} = "Cortex-M4.fp"
        LDR     R0, =0xE000ED88           ; Enable CP10,CP11
        LDR     R1,[R0]
        ORR     R1,R1,#(0xF << 20)
        STR     R1,[R0]
        ENDIF

        B       __main	    
;******************************************************************************
;
; This is the code that gets called when the processor receives a NMI.  This
; simply enters an infinite loop, preserving the system state for examination
; by a debugger.
;
;******************************************************************************
NmiSR
        B       NmiSR

;******************************************************************************
;
; This is the code that gets called when the processor receives a fault
; interrupt.  This simply enters an infinite loop, preserving the system state
; for examination by a debugger.
;
;******************************************************************************
FaultISR
        B       FaultISR

;******************************************************************************
;
; This is the code that gets called when the processor receives an unexpected
; interrupt.  This simply enters an infinite loop, preserving the system state
; for examination by a debugger.
;
;******************************************************************************
IntDefaultHandler
        B       IntDefaultHandler

;******************************************************************************
;
; Make sure the end of this section is aligned.
;
;******************************************************************************
        ALIGN

;******************************************************************************
;
; Some code in the normal code section for initializing the heap and stack.
;
;******************************************************************************
        AREA    |.text|, CODE, READONLY

;******************************************************************************
;
; The function expected of the C library startup code for defining the stack
; and heap memory locations.  For the C library version of the startup code,
; provide this function so that the C library initialization code can find out
; the location of the stack and heap.
;
;******************************************************************************
    IF :DEF: __MICROLIB
        EXPORT  __initial_sp
        EXPORT  __heap_base
        EXPORT __heap_limit
    ELSE
        IMPORT  __use_two_region_memory
        EXPORT  __user_initial_stackheap
__user_initial_stackheap
        LDR     R0, =HeapMem
        LDR     R1, =(StackMem + Stack)
        LDR     R2, =(HeapMem + Heap)
        LDR     R3, =StackMem
        BX      LR
    ENDIF

;******************************************************************************
;
; Make sure the end of this section is aligned.
;
;******************************************************************************
        ALIGN

;******************************************************************************
;
; Tell the assembler that we're done.
;
;******************************************************************************
        END
