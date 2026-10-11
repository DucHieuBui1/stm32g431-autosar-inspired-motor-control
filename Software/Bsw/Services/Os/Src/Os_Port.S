.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.equ OS_KERNEL_BASEPRI, 0x80

.extern Os_PortGetCurrentStackPointer
.extern Os_PortSaveCurrentStackPointer
.extern Os_PortSelectNextStackPointer
.extern _estack

.section .text.Os_PortStartFirstTask,"ax",%progbits
.global Os_PortStartFirstTask
.type Os_PortStartFirstTask, %function
.thumb_func
Os_PortStartFirstTask:
    svc     0
1:
    b       1b
.size Os_PortStartFirstTask, .-Os_PortStartFirstTask

.section .text.SVC_Handler,"ax",%progbits
.global SVC_Handler
.type SVC_Handler, %function
.thumb_func
SVC_Handler:
    push    {r3, lr}
    bl      Os_PortGetCurrentStackPointer
    pop     {r3, lr}
    ldmia   r0!, {r4-r11}
    msr     psp, r0
    /* The abandoned StartOS/SVC frame must not consume ISR stack space. */
    ldr     r1, =_estack
    msr     msp, r1
    movs    r1, #0
    msr     basepri, r1
    dsb
    isb
    ldr     lr, =0xFFFFFFFD
    bx      lr
.size SVC_Handler, .-SVC_Handler

.section .text.PendSV_Handler,"ax",%progbits
.global PendSV_Handler
.type PendSV_Handler, %function
.thumb_func
PendSV_Handler:
    movs    r1, #OS_KERNEL_BASEPRI
    msr     basepri, r1
    dsb
    isb
    mrs     r0, psp
    stmdb   r0!, {r4-r11}
    push    {r3, lr}
    bl      Os_PortSaveCurrentStackPointer
    bl      Os_PortSelectNextStackPointer
    pop     {r3, lr}
    ldmia   r0!, {r4-r11}
    msr     psp, r0
    movs    r1, #0
    msr     basepri, r1
    dsb
    isb
    bx      lr
.size PendSV_Handler, .-PendSV_Handler
