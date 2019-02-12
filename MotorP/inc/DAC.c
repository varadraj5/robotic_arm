/*
 * PWM DAC or TLV5638 driver implementation.
 * Created by Jun Ouyang (jdouyang@ucdavis.edu)
 * Nov 22, 2018
 *
 * Simplified BSD License (FreeBSD License)
 * Copyright (c) 2018, Jun Ouyang, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 */
#include "DAC.h"
#include "CortexM.h"
#include "msp.h"
#include "LaunchPad.h"
#include "Util.h"
#include <math.h>

void DAC_PWM_AnalogWrite(uint16_t val, uint8_t capComModule);
void DAC_TLV5638_AnalogWrite(uint16_t val, uint16_t capComModule);
void DAC_TLV5638_AnalogWrite_WriteWord(uint16_t val);


void DAC_Init()
{
    switch (DAC_MODE)
    {
    case DAC_MODE_PWM:
    {
        DAC_TimerA2_Init();
    }
    case DAC_MODE_TLV5638:
    {
        DAC_TLV5638_Init();
    }
    }

}

void DAC_TimerA2_Init()
{

    TIMER_A2->R = 0x0000;
//        TIMER_A2->CCTL[0] = 0x0080;      // CCI0 toggle
    TIMER_A2->CCR[0] = DAC_PWM_MAX;   // Period is 24000 cycles*83.3ns = 2ms
    TIMER_A2->EX0 = 0x0000;        //    divide by 1
    TIMER_A2->CTL = 0x0210;        // SMCLK=12MHz, up mode
                                   //0000_0010_0001_0000
}

void DAC_TLV5638_Init(){
    //Set P1.5, P1.6 and P1.7 for UCB0 functions
    P1->SEL0 |= 0xE0; //1110_0000
    P1->SEL1 &= ~0xE0; //0000_0000

    //CS
    P4->SEL0 &= ~0x04;
    P4->SEL1 &= ~0x04;
    P4->DIR |= 0x04;

    P4->OUT |= 0x04;

    //Initialize EUSCIB0
    EUSCI_B0->CTLW0 |= 0x0001; //hold in reset state

    //UCBxCTLW0
    /*
     * UCCKPH = 1 //data changed on first UCLK edge?
     * UCCKPL = 1 //clock inactive state is high
     * UCMSB = 1 //MSB first, in addition, TLV5638 shall be little endian
     * UC7BIT = 0 //8bit
     * UCMST = 1 //master mode
     * UCMODE0 = 00 //3-pin SPI
     * UCSYNC = 1 //Synchronous mode
     * UCSSEL0 = 10 //SMCLK
     * Reserved = xxxx
     * UCSTEM = 0 // Ignored since 3wire mode
     * UCSWRST = 1 // held in reset state for config
     * = 0110_1001_10xx_xx01 -> 0xE981
     */
    EUSCI_B0->CTLW0 = 0xE981;

    EUSCI_B0->BRW = 0x0000; //no division, use full 12MHz as SCLK.

    //setup interrupts for TX (ignores P1.7 for the moment being)

    EUSCI_B0->IE &= ~0x0003; //disables TX/RX interrupt.
//    EUSCI_B0->IE |= 0x0002;
    //Interrupt 20 on NVIC
//    NVIC->ISER[0] |= 0x00100000;
//    NVIC->IP[5] |= 0x00000000; //highest priority for interrupt 20.

    EUSCI_B0->CTLW0 &= ~0x0001; //release for operation

    //Setup TLV5638 reference voltage
    /*
     * Fast mode: SPD = 1
     * Power on: PWR = 0
     * Config: R1 = 1, R0 = 1
     * D1 = 0, D0 = 0 (External reference = 1.65V)
     * -> 1101_0000_0000_0010 = 0xD002
     */
    DAC_TLV5638_AnalogWrite_WriteWord(0xD000);

    //write 1.65V for both A and B (simultaneously)
    //12 bits max = 4095, max = 4095>>1 = 2047 = 0x7FF
    /*
         * Fast mode: SPD = 1
         * Power on: PWR = 0
         * Config: R1 = 0, R0 = 1 (Write to buffer)
         * -> Prefix = 0101 = 0x5
         */
    DAC_TLV5638_AnalogWrite_WriteWord(((0x5 << 12) | 0x7FF));

    /*
             * Fast mode: SPD = 1
             * Power on: PWR = 0
             * Config: R1 = 1, R0 = 0 (Write to A, move buffer to B)
             * -> Prefix = 1100 = 0xC
             */
    DAC_TLV5638_AnalogWrite_WriteWord(((0xC << 12) | 0x7FF));
}

uint32_t DAC_Max()
{
    return (DAC_MODE? DAC_TLV5638_MAX : DAC_PWM_MAX);
}

/**
 * capComModule:
 * For PWM:
 *  Specifies the module Ax to write to.
 *
 * For TLV5638:
 * 0: Writes to first Channel.
 * 1: Writes to second channel.
 */
void DAC_AnalogWrite(uint16_t val, uint8_t capComModule)
{
    switch(DAC_MODE){
    case DAC_MODE_PWM:
    {
     DAC_PWM_AnalogWrite(val, capComModule);
    }
    case DAC_MODE_TLV5638:
    {
        DAC_TLV5638_AnalogWrite(val, capComModule);
    }
    }

}

void DAC_PWM_AnalogWrite(uint16_t val, uint8_t capComModule){
    if (val > DAC_PWM_MAX)
        return;

    TIMER_A2->CCTL[capComModule] = 0x00C0;      // reset/set
    TIMER_A2->CCR[capComModule] = val;
}

//No FIFO since SCLK is 12MHz, more than sufficient for a refresh rate of only 1/0.1ms = 10kHz at max.
//This means no need for interrupt to dequeue. Polls UCTXIFG is best on write.
void DAC_TLV5638_AnalogWrite(uint16_t val, uint16_t capComModule){
    uint8_t txReady = EUSCI_B0->IFG & 0x0002;

    val &= 0x0FFF; //only accepts 12 bits.

    //0: DAC A, 1: DAC B
    //A: 1100
    //B: 0100
    uint16_t prefix = ((capComModule? 0x4:0xC) << 12);

    DAC_TLV5638_AnalogWrite_WriteWord((prefix | val));
}

void DAC_TLV5638_AnalogWrite_WriteWord(uint16_t val){

    P4->OUT &= ~0x04;
    //Write MSB first
    while ((EUSCI_B0->IFG & 0x0002) == 0x0000); //poll until UCTXIFG = 1

    uint8_t msb = (uint8_t) (val >> 8);

            EUSCI_B0->TXBUF = msb; //byte 1

    //Write LSB
    while ((EUSCI_B0->IFG & 0x0002) == 0x0000); //poll until UCTXIFG = 1

    uint8_t lsb = (uint8_t) (val & 0x00FF);

        EUSCI_B0->TXBUF = lsb; //byte 0

    while (EUSCI_B0->STATW & 0x0001); //de-assert only after finished transmission.
    P4->OUT |= 0x04;
}
