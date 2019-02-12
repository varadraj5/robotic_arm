/*
 * Ponolu Romi Motor driver header.
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

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define VEHICLE_PWM_PERIOD 1200 //total number of clock cycles per PWM waveform period.
#define WHEEL_CIRCUMFERENCE 0.2199 // 70mm*pi, circumference of wheel
#define EDGES_PER_WHEEL_REVOLUTION 720 // 1440 with posedge and negedge
#define DISTANCE_BETWEEN_EDGE 0.000305432619099 // distance between detection.

//define your own global variables here
uint32_t motorStartSysTickCountLeft;
uint32_t motorStartSysTickCountRight;

void vehicle_init();
void vehicle_move_forward(uint16_t,uint16_t);
void vehicle_move_backward(uint16_t,uint16_t);
void vehicle_turn_left(uint16_t,uint16_t);
void vehicle_turn_right(uint16_t,uint16_t);
void vehicle_rotate_left(uint16_t,uint16_t);
void vehicle_rotate_right(uint16_t,uint16_t);
void vehicle_run_left(uint16_t, uint8_t);
void vehicle_run_right(uint16_t, uint8_t);
void vehicle_stop_left();
void vehicle_stop_right();
void vehicle_stop();
uint32_t ms2count(uint32_t ms);

void PWM_Init12(uint16_t period, uint16_t duty1, uint16_t duty2);
void PWM_Duty1(uint16_t duty1);
void PWM_Duty2(uint16_t duty2);

void TimerA2_Init(uint16_t period);

#endif
