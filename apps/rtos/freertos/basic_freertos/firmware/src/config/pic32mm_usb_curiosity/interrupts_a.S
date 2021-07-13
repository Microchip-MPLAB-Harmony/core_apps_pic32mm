/*******************************************************************************
  System Interrupts File

  Company:
    Microchip Technology Inc.

  File Name:
    interrupt_a.S

  Summary:
    Raw ISR definitions.

  Description:
    This file maps all the interrupt vectors to their corresponding
    implementations. If a particular module interrupt is used, then its ISR
    definition can be found in corresponding PLIB source file. If a module
    interrupt is not used, then its ISR implementation is mapped to dummy
    handler.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include "ISR_Support.h"

   .extern  CHANGE_NOTICE_B_Handler

   .section .vector_9,code, keep
   .equ     __vector_dispatch_9, IntVectorCHANGE_NOTICE_B_Handler
   .global  __vector_dispatch_9
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorCHANGE_NOTICE_B_Handler

IntVectorCHANGE_NOTICE_B_Handler:
    j  _CHANGE_NOTICE_B_Handler
    nop
    .end    IntVectorCHANGE_NOTICE_B_Handler

   .section .CHANGE_NOTICE_B_Handler_vector_text, code, keep
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _CHANGE_NOTICE_B_Handler

_CHANGE_NOTICE_B_Handler:
    portSAVE_CONTEXT
    la    s6,  CHANGE_NOTICE_B_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end    _CHANGE_NOTICE_B_Handler
   .extern  UART3_RX_Handler

   .section .vector_59,code, keep
   .equ     __vector_dispatch_59, IntVectorUART3_RX_Handler
   .global  __vector_dispatch_59
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorUART3_RX_Handler

IntVectorUART3_RX_Handler:
    j  _UART3_RX_Handler
    nop
    .end    IntVectorUART3_RX_Handler

   .section .UART3_RX_Handler_vector_text, code, keep
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _UART3_RX_Handler

_UART3_RX_Handler:
    portSAVE_CONTEXT
    la    s6,  UART3_RX_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end    _UART3_RX_Handler
   .extern  UART3_TX_Handler

   .section .vector_60,code, keep
   .equ     __vector_dispatch_60, IntVectorUART3_TX_Handler
   .global  __vector_dispatch_60
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorUART3_TX_Handler

IntVectorUART3_TX_Handler:
    j  _UART3_TX_Handler
    nop
    .end    IntVectorUART3_TX_Handler

   .section .UART3_TX_Handler_vector_text, code, keep
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _UART3_TX_Handler

_UART3_TX_Handler:
    portSAVE_CONTEXT
    la    s6,  UART3_TX_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end    _UART3_TX_Handler
   .extern  UART3_ERR_Handler

   .section .vector_61,code, keep
   .equ     __vector_dispatch_61, IntVectorUART3_ERR_Handler
   .global  __vector_dispatch_61
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorUART3_ERR_Handler

IntVectorUART3_ERR_Handler:
    j  _UART3_ERR_Handler
    nop
    .end    IntVectorUART3_ERR_Handler

   .section .UART3_ERR_Handler_vector_text, code, keep
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _UART3_ERR_Handler

_UART3_ERR_Handler:
    portSAVE_CONTEXT
    la    s6,  UART3_ERR_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end    _UART3_ERR_Handler

