/*
 * Ponolu Romi Motor driver implementation.
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

#include "Motor.h"
#include "CortexM.h"
#include "msp.h"
#include "LaunchPad.h"
#include <math.h>

/*
 * No enough hardware support for timed movement mode. (Not enough timer). Using untimed movements.
 */
void vehicle_move_forward(uint16_t duty1, uint16_t duty2)
{
    DisableInterrupts();
    P3->OUT |= 0xC0;
    P6->OUT &= ~0x03;
    PWM_Duty1(duty1);
    PWM_Duty2(duty2);
    EnableInterrupts();

    uint32_t tick = SysTick->VAL;
    motorStartSysTickCountLeft = tick;
    motorStartSysTickCountRight = tick;

}

void vehicle_move_backward(uint16_t duty1, uint16_t duty2)
{
    DisableInterrupts();
    P3->OUT |= 0xC0;
    P6->OUT |= 0x03;
    PWM_Duty1(duty1);
    PWM_Duty2(duty2);
    EnableInterrupts();

    uint32_t tick = SysTick->VAL;
    motorStartSysTickCountLeft = tick;
    motorStartSysTickCountRight = tick;
}

void vehicle_turn_left(uint16_t duty1, uint16_t duty2)
{
    DisableInterrupts();
    P3->OUT |= 0xC0;
    P6->OUT &= ~0x02;
    PWM_Duty1(0);
    PWM_Duty2(duty2);
    EnableInterrupts();

    uint32_t tick = SysTick->VAL;
    motorStartSysTickCountRight = tick;

}

void vehicle_turn_right(uint16_t duty1, uint16_t duty2)
{
    DisableInterrupts();
    P3->OUT |= 0xC0;
    P6->OUT &= ~0x01;
    PWM_Duty1(duty1);
    PWM_Duty2(0);
    EnableInterrupts();

    uint32_t tick = SysTick->VAL;
    motorStartSysTickCountLeft = tick;

}

void vehicle_rotate_left(uint16_t duty1, uint16_t duty2)
{
    DisableInterrupts();
    P3->OUT |= 0xC0;
    P6->OUT &= ~0x03;
    P6->OUT |= 0x01;
    PWM_Duty1(duty1);
    PWM_Duty2(duty2);
    EnableInterrupts();

    uint32_t tick = SysTick->VAL;
    motorStartSysTickCountLeft = tick;
    motorStartSysTickCountRight = tick;
}

void vehicle_rotate_right(uint16_t duty1, uint16_t duty2)
{
    DisableInterrupts();
    P3->OUT |= 0xC0;
    P6->OUT &= ~0x03;
    P6->OUT |= 0x02;
    PWM_Duty1(duty1);
    PWM_Duty2(duty2);
    EnableInterrupts();

    uint32_t tick = SysTick->VAL;
    motorStartSysTickCountLeft = tick;
    motorStartSysTickCountRight = tick;
}

void vehicle_stop_left()
{
//    P1->OUT |= 0x40;
    PWM_Duty1(0);
}

void vehicle_stop_right()
{
//    P1->OUT |= 0x80;
    PWM_Duty2(0);
}

void vehicle_stop()
{
    vehicle_stop_left();
    vehicle_stop_right();
    P3->OUT &= ~0xC0;

//    DisableInterrupts(); //possible sensor measurement issues if left interrupt listening on posedge of Hall sensors.
}

void vehicle_GPIO_init()
{
    //P1.7(left DIR), P1.6(right DIR)
    P6->SEL0 &= ~0x03;
    P6->SEL1 &= ~0x03; // choose P6.0 and P6.1 as GPIO
    P6->DIR |= 0x03; // choose P6.0 and P6.1 as outputs
    P6->IE &= 0x00; // disables interrupts on P6

    //P3.7(left SLP), P3.6(right SLP)
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0; // choose P3.6 and P3.7 as GPIO
    P3->DIR |= 0xC0; // choose P3.6 and P3.7 as outputs
    P3->IE &= 0x00; // disables interrupts on P1
}

void vehicle_run_left(uint16_t duty, uint8_t dir){
    P3->OUT |= 0xC0;

        if (dir)
            P6->OUT |= 0x01;
        else
            P6->OUT &= ~0x01;
        PWM_Duty1(duty);
}

void vehicle_run_right(uint16_t duty, uint8_t dir){
    P3->OUT |= 0xC0;

        if (dir)
            P6->OUT |= 0x02;
        else
            P6->OUT &= ~0x02;
    PWM_Duty2(duty);
}

void vehicle_init()
{
    /*Period is 2*VEHICLE_PWM_PERIOD*41.66ns = 83.33ns*600 = 50us
     *same speed as DAC but output interleaves to conserve battery.
     */
    PWM_Init12(VEHICLE_PWM_PERIOD, 0, 0);
    vehicle_GPIO_init();
//    TimerA2_Init(50); // period = 50*2us = 100us => sampling frequency 10KHz
    vehicle_stop();
}


//define additional movement functions here.

//***************************PWM_Init12*******************************
// PWM outputs on P2.4, P2.5
// Inputs:  period (1.333us)
//          duty1
//          duty2
// Outputs: none
// SMCLK = 48MHz/4, 83.33ns
// Counter counts up to TA0CCR0 and back down
// Let Timerclock period T = 8/12MHz = 666.7ns
// P2.4=1 when timer equals TA0CCR1 on way up, P2.4=0 when timer equals TA0CCR1 on way down
// P2.5=1 when timer equals TA0CCR2 on way down, P2.5=0 when timer equals TA0CCR2 on way up
// Period of P2.4 is 166.67ns*period, duty cycle is duty1/period
// Period of P2.5 is 166.67ns*period, duty cycle is duty2/period
void PWM_Init12(uint16_t period, uint16_t duty1, uint16_t duty2)
{
    if (duty1 >= period)
        return; // bad input
    if (duty2 >= period)
        return; // bad input
    P2->DIR |= 0x30;          // P2.4, P2.5 output
    P2->SEL0 |= 0x30;         // P2.4, P2.5 Timer0A functions
    P2->SEL1 &= ~0x30;        // P2.4, P2.5 Timer0A functions
//    TIMER_A0->CCTL[0] = 0x0080;      // CCI0 toggle
    TIMER_A0->CCR[0] = period;   // Period is 2*period*83.33ns is 166.67ns*period
    TIMER_A0->EX0 = 0x0000;        //    divide by 1
    TIMER_A0->CCTL[1] = 0x00C0;      // CCR1 toggle/set
    TIMER_A0->CCR[1] = period-duty1;  // CCR1 duty cycle is duty1/period
    TIMER_A0->CCTL[2] = 0x0040;      // CCR2 toggle/reset
    TIMER_A0->CCR[2] = duty2;        // CCR2 duty cycle is duty2/period
    TIMER_A0->CTL = 0x0230;        // SMCLK=12MHz, up-down mode, 0000 0010 0011 0000
// bit  mode
// 9-8  10    TASSEL, SMCLK=12MHz
// 7-6  11    ID, divide by 8
// 5-4  11    MC, up-down mode
// 2    0     TACLR, no clear
// 1    0     TAIE, no interrupt
// 0          TAIFG
}

//***************************PWM_Duty1*******************************
void PWM_Duty1(uint16_t duty1)
{
    uint16_t period = TIMER_A0->CCR[0];
    if (duty1 >= period)
        return; // bad input
    TIMER_A0->CCR[1] = period - duty1;        // CCR1 duty cycle is duty1/period
}

//***************************PWM_Duty2*******************************
// change duty cycle of PWM output on P2.5
// Inputs:  duty2
// Outputs: none// period of P2.5 is 2*period*666.7ns, duty cycle is duty2/period
void PWM_Duty2(uint16_t duty2)
{
    if (duty2 >= TIMER_A0->CCR[0])
        return; // bad input
    TIMER_A0->CCR[2] = duty2;        // CCR2 duty cycle is duty2/period
}

/***********************************************************************************
 * End vehicle movement functions.
 ***********************************************************************************/

uint32_t ms2count(uint32_t ms)
{
    return ms * 10;
}
