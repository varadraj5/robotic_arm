/*
 * 14-bit ADC driver header.
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

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdint.h>

void ADC14_Init(void); //Initialize cores. Clears end of sequence bits.

void ADC14_AddInput(uint8_t mem, uint8_t channel); //Add Ax pin to mem

uint16_t ADC14_GetResult(uint8_t mem); //Gets result from specific mem

void ADC14_BeginConversion(); //Begins conversion.

void ADC14_WaitConversion(); //Wait on all channels

void ADC14_WaitConversionAtMem(uint8_t mem); //Wait on only one channel

void ADC14_EndConversion(); //End conversion. Sets end of sequence bits to all mem cores.

void ADC14_EndConversionAtMem(uint8_t mem);


#endif /* INC_ADC_H_ */
