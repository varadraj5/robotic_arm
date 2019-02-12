/*
 * 14-bit ADC driver implementation.
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
#include "ADC.h"
#include "CortexM.h"
#include "msp.h"
#include "LaunchPad.h"
#include "Util.h"
#include <math.h>

/**
 * ADC sampling using SMCLK = HFXT/2 clock at 12MHz. It takes 32 cycles for sampling. 1 cycle to start converting.
 * 16 cycles to convert. 1 cycle to move data to memory. Total = 50 cycles. With a 12MHz clock, this is
 * equivalent to 12MHz/50 = 240KHz sampling rate.
 */

uint8_t mems[32];
uint32_t memMask;

void ADC14_Init(void)
{

    for (int i = 0; i < 32; i++)
    { // avoid stdio inclusion, use simple loop instead
        mems[i] = 0;
    }
    memMask = 0x00000000;
    ADC14->CTL0 &= ~0x00000002;      // 2) ADC14ENC = 0 to allow programming
    while (ADC14->CTL0 & 0x00010000)
    {
    };
    ADC14->CTL0 = 0x04260390;  //ADC14PDIV = 00b (/1), ADC14SHSx = 000b (ADC14SC)
                              //ADC14SHP = 1b (SAMPCON sourced from sampling timer, Pulsed mode)
                              //ADC14ISSH = 0b, ADC14DIVx = 000b (/1)
                              //ADC14SSELx = 100b (HSMCLK), ADC14CONSEQx = 11b (Repeated Sequence of Channels)
                              //ADC14BUSY = 0b, ADC14SHT1x = 0000b (not used), ADC14SHT0x = 0000b (not used)
                              //ADC14MSC = 1b (uniform sampling + conversion?)
                              //Reserved = 00b, ADC14ON = 1b (Turns on ADC14)
                              //Reserved = 00b, ADC14ENC = 0b (Disables ADC14 Conversion)
                              //ADC14SC = 0b (Do not begin conversion yet)
                              // = 0000_0100_0010_0110_0000_0011_1001_0000 = 0x04260390 (Modified)
    ADC14->CTL1 = 0x00000030;

    ADC14->IER0 = 0;          //no interrupts
    ADC14->IER1 = 0;
}

void ADC14_AddInput(uint8_t mem, uint8_t channel)
{
    mems[mem] = channel;
    memMask |= 0x1<<mem;
    ADC14->MCTL[mem] = (0x00000000 | (channel & 0x1F));
    //Reserved = 0x0000
    //ADC14WINCTH = 0b (Use window comparator thresholds)
    //ADC14WINC = 0b (Comparator window disabled)
    //ADC14DIF = 0b (Single-ended input mode)
    //Reserved = 0b, ADC14VRSEL = 0000b (VR+ = AVCC = 3.3V, VR- = AVSS = 0V)
    //ADC14EOS = 0b (Not end of sequence)
    //Reserved = 00b, ADC14INCHx = channel & 0x1F = c4c3c2c1c0
    // = 0000_0000_0000_0000_0000_0000_000c4_c3c2c1c0

}

uint16_t ADC14_GetResult(uint8_t mem)
{
    ADC14_WaitConversionAtMem(mem);
    return ADC14->MEM[mem];
}

void ADC14_BeginConversion()
{
    ADC14->CTL0 &= ~0x00000001;
    ADC14->CTL0 |= 0x00000002; // ADC14ENC = 1b
    sleep(10); //delays 10ms before setting it high to give posedge
    ADC14->CTL0 |= 0x00000001; //ADC14SC = 1b
}

void ADC14_WaitConversion()
{
    while ((ADC14->CTL0 & 0x00010000) || ((ADC14->IFGR0 & memMask) == 0))
    {
    };
}

uint32_t ADC14CTL0 = 0;
uint32_t ADC14IFGR0 = 0;

void ADC14_WaitConversionAtMem(uint8_t mem)
{
    while ((ADC14->IFGR0 & (0x1<<mem)) == 0)
    {
//        ADC14CTL0 = ADC14->CTL0;
//        ADC14IFGR0 = ADC14->IFGR0;
    };
}

void ADC14_EndConversion()
{
    //set end of sequences
    int i = 0;
    while (i < 32 && mems[i] > 0)
    {
        ADC14->MCTL[i] |= 0x00000080;
    }
    //ADC14SC = 0
    ADC14->CTL0 &= ~0x00000001;
}

void ADC14_EndConversionAtMem(uint8_t mem){
    ADC14->MCTL[mem] |= 0x00000080;
}

