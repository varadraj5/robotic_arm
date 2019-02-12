/**
 * RSLK Proportional Controller main entry file.
 * Created by Jun Ouyang
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

#include "msp.h"
#include "Clock.h"
#include "LaunchPad.h"
#include "Motor.h"
#include "ADC.h"
#include "DAC.h"
#include "Util.h"
#include "CortexM.h"
#include "EUSCIA0.h"
#include "SysTick.h"
#include <stdint.h>
#include <math.h>

#define GAIN_BUDGET_LEFT 40.0 //Kmax
#define GAIN_BUDGET_RIGHT 40.0 //Kmax

#define DISTANCE 1.0 //Desired distance. Must than less than 1m

/**
 * -1: No UART transmission.
 * 0: Left wheel distance will be transmitted via USB UART
 * 2: Left wheel ADC output will be transmitted via USB UART
 * 3: Right wheel ADC output will be transmitted via USB UART
 */
#define DISTANCE_PROFILE 0

/* DO NOT MODIFY THE REMAINING SECTIONS UNLESS YOU KNOW WHAT YOU ARE DOING */

/**
 * Pin Map.
 * !Unless specified otherwise, the list follows (left), (right) format.
 *
 * Motor PWM: P2.4, P2.5
 * Motor DIR: P6.0, P6.1
 * Motor EN: P3.6, P3.7
 * ADC: P5.4, P5.5
 * DAC: P5.6, P5.7
 * Encoder A: P4.0, P5.0
 * Encoder B: P4.1, P5.1
 *
 * UCB0SIMO: P1.6
 * UCB0SOMI: P1.7
 * UCB0CLK: P1.5
 * UCB0CS: P4.2
 */

/*
 * DAC Remark:
 *
 * We want to output signal from DAC such that GAIN_BUDGET times the output is still within the ADC limit.
 *
 * Let GAIN_BUDGET = 100. This means the max voltage is 3.3/100 = 0.033 = 33mV.
 *
 * For SMCLK, because the maximum it can go is half of HMCLK = 48MHz and also the optimal setting for UART at 115200Hz is at SMCLK = 12MHz.
 * We are forced to use SMCLK = 12MHz...
 *
 * For a 1ms data acquisition rate from application (set by TA1), with SMCLK = 12MHz, the PWM timer
 * can count 12MHz*1ms = 12,000 times. For a 5 cycle PWM, each cycle has 12,000/5 = 2,400 counts.
 * LSB is 3.3/16384 ~ 0.20mV.
 * With maximum 33mV, this means there are 33/0.20 = 165 voltage levels. +82 and -82. 1 for 0. This is not so accurate.
 *
 * In addition, because of PWM filter settling time vs. accuracy issue, it is recommended to use an actual DAC over the use of PWM DAC.
 * The code ships with the TLV5638 driver. See DAC.h.
 */

/*
 * Motor PWM Remark:
 * Motor PWM uses TA2 that bases off of SMCLK. With 1200 counts, the PWM period is 1200/12MHz = 0.1ms.
 */

/***********************************************************************************
 * Parameters and global variables. DO NOT MODIFY UNLESS YOU KNOW WHAT YOU ARE DOING
 ***********************************************************************************/

#define MAX_DISTANCE 1.0
#define LOOP_DELAY 1000 //In terms of microseconds. LOOP_DELAY must be greater and equal than 1000
#define ADC_VDD 3.3
#define ADC_VSS 0.0
#define ADC_BIAS 1.65

float left_distance = 0;
float right_distance = 0;

float left_distance_error = 0;
float right_distance_error = 0;

uint16_t DAC_PWM_Left = 0;
uint16_t DAC_PWM_Right = 0;

float ADC_PID_Left = 0;
float ADC_PID_Right = 0;

uint16_t VEHICLE_PWM_Left = 0;
uint16_t VEHICLE_PWM_Right = 0;

float ADC_BIAS_LEFT = 1.65;
float ADC_BIAS_RIGHT = 1.65;
float ADC_VDD_LEFT = 3.3;
float ADC_VDD_RIGHT = 3.3;
float ADC_VSS_LEFT = 0.0;
float ADC_VSS_RIGHT = 0.0;

uint8_t CALIBRATE_LEFT = 0;
uint8_t CALIBRATE_RIGHT = 0;
int8_t DIR_FORWARD_LEFT = 0;
int8_t DIR_FORWARD_RIGHT = 0;

void enc_GPIO_init();
void TA1_Init();

void updateLeftError();
void updateRightError();
void updateLeftADC();
void updateRightADC();
void updateLeftCommand();
void updateRightCommand();
void calibrateLeft();
void calibrateRight();

void enc_GPIO_init()
{
    //setup P4.0 and P5.0 to listen on interrupt, P4.1 and P5.1 as input B for direction
    P4->DIR &= ~0x03; //Configure P4.0, P4.1 as input pin.

    P4->SEL0 &= ~0x03;
    P4->SEL1 &= ~0x03; //Configure P4.0 as GPIO.

    P4->REN |= 0x03; //Pulling enabled
    P4->OUT |= 0x03; //Pull up

    P4->IES &= ~0x01; // Configure P4.0 to listen on posedge.
    P4->IE |= 0x01; //Enable P4.0 Interrupt.
    P4->IFG &= ~0x01; //Clears P4.0 interrupt.

    NVIC->ISER[1] |= 0x00000040; //Enable interrupt 38 (P4)
    NVIC->IP[9] &= 0xFF00FFFF;
    NVIC->IP[9] |= 0xFF00FFFF; //Set highest priority

    P5->DIR &= ~0x03; //Configure P5.0, P5.1 as input pin.

    P5->SEL0 &= ~0x03;
    P5->SEL1 &= ~0x03; //Configure P5.0, P5.1 as GPIO.

    P5->REN |= 0x03; //Pulling enabled
    P5->OUT |= 0x03; //Pull up

    P5->IES &= ~0x01; // Configure P5.0 to listen on posedge.
    P5->IE |= 0x01; //Enable P5.0 Interrupt.
    P5->IFG &= ~0x01; //Clears P5.0 interrupt.

    NVIC->ISER[1] |= 0x00000080; //Enable interrupt 39 (P5)
    NVIC->IP[9] &= 0x00FFFFFF;
    NVIC->IP[9] |= 0x00FFFFFF; //Set highest priority
}

void TA1_Init()
{
    TIMER_A1->R = 0x0000;
    TIMER_A1->CCR[0] = LOOP_DELAY; // Period is LOOP_DELAY*1us. Must be > 1ms for the system to function properly.
    TIMER_A1->EX0 = 0x0002; //0x0002;        //    3MHz/3=1MHz
    TIMER_A1->CTL = 0x0292;        // SMCLK=12MHz/4 = 3MHz, up mode
                                   //0000_0010_1001_0010
    TIMER_A1->CCTL[0] = 0x0010;
    NVIC->IP[2] = (NVIC->IP[2] & 0xFF00FFFF) | 0x00020000;
    NVIC->ISER[0] |= 0x00000400; //INTISR[10] = TA1CCTL0.CCIFG, 0000_0000_0000_0000_0000_0100_0000_0000
}

void calibrateLeft()
{
    DAC_AnalogWrite(DAC_Max(), 0); //Obtains maximum voltage
    sleep(1);
    ADC_VDD_LEFT = ADC14_GetResult(0) / 16384.0 * ADC_VDD;

    DAC_AnalogWrite(0, 0); //Obtains min-point voltage
    sleep(1);
    ADC_VSS_LEFT = ADC14_GetResult(0) / 16384.0 * ADC_VDD;

    DAC_AnalogWrite(DAC_Max() >> 1, 0); //Obtains half-point voltage
    sleep(1);
    ADC_BIAS_LEFT = ADC14_GetResult(0) / 16384.0 * ADC_VDD;

    //Detect forward direction
    vehicle_run_left(VEHICLE_PWM_PERIOD >> 1, 0);
    CALIBRATE_LEFT = 1;
    while (CALIBRATE_LEFT)
        ;
    vehicle_stop_left();
}

void calibrateRight()
{
    DAC_AnalogWrite(DAC_Max(), 1); //Obtains maximum voltage
    sleep(1);
    ADC_VDD_RIGHT = ADC14_GetResult(1) / 16384.0 * ADC_VDD;

    DAC_AnalogWrite(0, 1); //Obtains min-point voltage
    sleep(1);
    ADC_VSS_RIGHT = ADC14_GetResult(1) / 16384.0 * ADC_VDD;

    DAC_AnalogWrite(DAC_Max() >> 1, 1); //Obtains half-point voltage
    sleep(1);
    ADC_BIAS_RIGHT = ADC14_GetResult(1) / 16384.0 * ADC_VDD;

    //Detect forward direction
    vehicle_run_right(VEHICLE_PWM_PERIOD >> 1, 0);
    CALIBRATE_RIGHT = 1;
    while (CALIBRATE_RIGHT)
        ;
    vehicle_stop_right();
}

void updateLeftError()
{
    /* Left motor */
    left_distance_error = DISTANCE - left_distance * GAIN_BUDGET_LEFT;
    //write to op amp
    DAC_PWM_Left = round(
            (0.5 + left_distance_error / 2.0 / GAIN_BUDGET_LEFT) * DAC_Max());
    DAC_AnalogWrite(DAC_PWM_Left, 0); //Write duty cycle to DAC A

}

void updateRightError()
{
    /* Right motor */
    right_distance_error = DISTANCE - right_distance * GAIN_BUDGET_RIGHT;
    //write to op amp
    DAC_PWM_Right = round(
            (0.5 + right_distance_error / 2.0 / GAIN_BUDGET_RIGHT) * DAC_Max());

    DAC_AnalogWrite(DAC_PWM_Right, 1); //Write duty cycle to DAC B

}

void updateLeftADC()
{
    ADC_PID_Left = ADC14_GetResult(0) / 16384.0 * (ADC_VDD_LEFT - ADC_VSS_LEFT)
            - ADC_BIAS_LEFT + ADC_VSS_LEFT; //Read from P5.4
}

void updateRightADC()
{
    ADC_PID_Right = ADC14_GetResult(1) / 16384.0
            * (ADC_VDD_RIGHT - ADC_VSS_RIGHT) - ADC_BIAS_RIGHT + ADC_VSS_RIGHT; //Read from P5.5
}

void updateLeftCommand()
{
    /********* Assigns new PWM command ************/
    uint8_t left_pwm_dir;
    //assumes op amp output has positive gain.
    if (ADC_PID_Left > 0.0)
    {
        VEHICLE_PWM_Left = round(
                fabs(ADC_PID_Left) / (ADC_VDD - ADC_BIAS) * VEHICLE_PWM_PERIOD);
        left_pwm_dir = 0; //forward
    }
    else
    {
        VEHICLE_PWM_Left = round(
                fabs(ADC_PID_Left) / (ADC_BIAS - ADC_VSS) * VEHICLE_PWM_PERIOD);
        left_pwm_dir = 1; //backward
    }

    vehicle_run_left(VEHICLE_PWM_Left, left_pwm_dir);

}

void updateRightCommand()
{
    /********* Assigns new PWM command ************/
    uint8_t right_pwm_dir;
    //assumes op amp output has positive gain.
    if (ADC_PID_Right > 0.0)
    {
        VEHICLE_PWM_Right =
                round(fabs(ADC_PID_Right)
                        / (ADC_VDD - ADC_BIAS)* VEHICLE_PWM_PERIOD);
        right_pwm_dir = 0;
    }
    else
    {
        VEHICLE_PWM_Right =
                round(fabs(ADC_PID_Right)
                        / (ADC_BIAS - ADC_VSS)* VEHICLE_PWM_PERIOD);
        right_pwm_dir = 1;
    }
    vehicle_run_right(VEHICLE_PWM_Right, right_pwm_dir);

}

void TA1_0_IRQHandler(void)
{
    TIMER_A1->CCTL[0] &= ~0x0001; //acknowledge

    updateLeftADC();
    updateRightADC();
    updateLeftCommand();
    updateRightCommand();
    updateLeftError();
    updateRightError();
}

void PORT4_IRQHandler()
{
    if (CALIBRATE_LEFT)
    {
        P4->IFG &= ~0x01; //Clears P4.0 interrupt.
        DIR_FORWARD_LEFT = ((P4->IN & 0x02) >> 1) ? -1 : 1;
        CALIBRATE_LEFT = 0;
        return;
    }
    /********* Detect feedback signal *************/
    P4->IFG &= ~0x01; //Clears P4.0 interrupt.
    uint8_t left_dir = ((P4->IN & 0x02) >> 1);

    //B input: 0V = LOW: forward, 3.3V = HIGH: backward
    left_distance += (left_dir ? -1 : 1) * DIR_FORWARD_LEFT
            * DISTANCE_BETWEEN_EDGE;
}

void PORT5_IRQHandler(void)
{
    if (CALIBRATE_RIGHT)
    {
        P5->IFG &= ~0x01; //Clears P5.0 interrupt.
        DIR_FORWARD_RIGHT = ((P5->IN & 0x02) >> 1) ? -1 : 1;
        CALIBRATE_RIGHT = 0;
        return;
    }
    /********* Detect feedback signal *************/
    P5->IFG &= ~0x01; //Clears P5.0 interrupt.

    uint8_t right_dir = ((P5->IN & 0x02) >> 1);

    //B input: 0V = LOW: backward, 3.3V = HIGH: forward
    right_distance += (right_dir ? -1 : 1) * DIR_FORWARD_RIGHT
            * DISTANCE_BETWEEN_EDGE;
}

/*
 * Use timer to periodically (~10ms) obtain speed data. Need second input encB to tell direction of the wheel.
 */

int main(void)
{
    if (DISTANCE > MAX_DISTANCE)
        return 1;

    Clock_Init48MHz(); // makes bus clock 48 MHz
    LaunchPad_Init(); // use buttons to step through frequencies

    ADC14_Init();
    ADC14_AddInput(0, 1); //MEM[0] = A1 = P5.4
    ADC14_AddInput(1, 0); //MEM[1] = A0 = P5.5
    P5->SEL0 |= 0x30;
    P5->SEL1 |= 0x30;

    //P5.6 capComModule = 1 (TA2.1), P5.7 capComModule = 2 (TA2.2)
    P5->DIR |= 0xC0;          // P5.6,P5.7 as outputs
    P5->SEL0 |= 0xC0;         // P5.6, P5.7 Timer2A functions
    P5->SEL1 &= ~0xC0;        // P5.6, P5.7 Timer2A functions
    DAC_Init();

    vehicle_init(); //initialize P2.4,2.5 and Timer0A for PWM output
    enc_GPIO_init(); //Sets P4.0 and P5.0 as input and enable interrupt to observe posedge.
    SysTick_Init(); //initialize SysTick timer.

    EUSCIA0_Init();

    EnableInterrupts();
    sleep(1000);

    ADC14_BeginConversion(); //Turns conversion on for ADC

    calibrateLeft();
    calibrateRight();

    sleep(1000);

    TA1_Init();

    uint8_t * base_ptr;
    switch (DISTANCE_PROFILE)
    {
    case 0:
        base_ptr = (uint8_t *) &left_distance;
        break;
    case 1:
        base_ptr = (uint8_t *) &right_distance;
        break;
    case 2:
        base_ptr = (uint8_t *) &ADC_PID_Left;
        break;
    case 3:
        base_ptr = (uint8_t *) &ADC_PID_Right;
        break;
    }

    /**
     * UART must accept header prefix FF and newline termination.
     */
    while (1)
    {
        if (DISTANCE_PROFILE > -1)
        {
            EUSCIA0_OutChar('F');
            EUSCIA0_OutChar('F');
            uint8_t* ptr = base_ptr;
            for (int i = 0; i < 4; i++)
            {
                EUSCIA0_OutChar(*ptr); //little endian
                ptr++;
            }
            EUSCIA0_OutChar('\n');
        }
        sleep(1);
    };
}
