/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 * MSP432 ADC14 -  Single Channel with External Reference
 *
 * Description: This very simple code example shows the use of an external
 * reference for use in the ADC14 module. A single channel sample/conversion
 * is setup on input A0 (P5.5) and the sample timer is used to continuously
 * store the result in a local variable. The value coming in VREF+ (P5.6) is
 * treated as the max value of the ADC and the value coming in VREF- (P5.7) is
 * treated as the minimum value.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P5.5  |<---- A0 In
 *            |            P5.6  |<---- VREF+
 *            |            P5.7  |<---- VREF-
 *            |                  |
 *
 ******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>

#include <string.h>

#include <stdbool.h>

volatile uint16_t adcResult,adcResult1;

int a,b=40,c=40,direction=0,direction_1=0,direction_2=0,loop=0, result;

/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig pwmConfig =
{
        //uint_fast16_t clockSource;
        TIMER_A_CLOCKSOURCE_SMCLK,
        //uint_fast16_t clockSourceDivider;
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        //uint_fast16_t timerPeriod;
        1300,
        //uint_fast16_t compareRegister;
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        //uint_fast16_t compareOutputMode;
        TIMER_A_OUTPUTMODE_RESET_SET,
        //uint_fast16_t dutyCycle;
        10
};

extern Timer_A_PWMConfig pwmConfig3 =
{
TIMER_A_CLOCKSOURCE_SMCLK,
TIMER_A_CLOCKSOURCE_DIVIDER_1,
1300,
TIMER_A_CAPTURECOMPARE_REGISTER_3,
TIMER_A_OUTPUTMODE_RESET_SET,
10
};


int main(void)
{
    /* Halting WDT */
    WDT_A_holdTimer();
    Interrupt_enableSleepOnIsrExit();
    
    //![Simple REF Example]
       /* Setting reference voltage to 2.5  and enabling reference */
       MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
       MAP_REF_A_enableReferenceVoltage();
       //![Simple REF Example]

    //![Simple ADC14 Configure]
    /* Initializing ADC (MCLK/1/1) */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
            0);

    /* Configuring ADC Memory (ADC_MEM0 A0/A1) in repeat mode
     * with use of external references */
    ADC14_configureSingleSampleMode(ADC_MEM0, true);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
            ADC_INPUT_A0, false);
    ADC14_configureSingleSampleMode(ADC_MEM1, true);
      // ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_EXTPOS_VREFNEG_EXTNEG,
         //      ADC_INPUT_A1, false);
    ADC14_configureConversionMemory(ADC_MEM1,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A1, false);

    /* Setting up GPIO pins as analog inputs (and references) */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
            GPIO_PIN7 | GPIO_PIN6 | GPIO_PIN5 | GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                GPIO_PIN7 | GPIO_PIN6, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Setting MCLK to REFO at 128Khz for LF mode setting SMCLK to 64Khz */
                    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
                    MAP_CS_initClockSignal(CS_MCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
                    MAP_CS_initClockSignal(CS_SMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_2);
                    MAP_PCM_setPowerState(PCM_AM_LF_VCORE0);

                    // Configure P6.6 for Timer_A2 Timer_A2.CCI3A_Out3
                        //
                        MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
                    // Configure P2.4 for Timer_A0 Timer_A0.CCI0A_Out0
                    //
                    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

                    // Configuring Timer_A to have a period of approximately 20ms
                    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
                    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig3);

    /* Enabling sample timer in auto iteration mode and interrupts*/
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    ADC14_enableInterrupt(ADC_INT1);
    ADC14_enableInterrupt(ADC_INT0);

    /* Enabling Interrupts */
    Interrupt_enableInterrupt(INT_ADC14);
    Interrupt_enableMaster();

    /* Triggering the start of the sample */
    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();
    //![Simple ADC14 Configure]



    /* Going to sleep */
    while (1)
    {
        PCM_gotoLPM0();
    }
}

/* This interrupt happens whenever a conversion has been completed and placed
 * into ADC_MEM0. */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);



    if(status & ADC_INT1)
    {
        adcResult = ADC14_getResult(ADC_MEM0);
       adcResult1 = ADC14_getResult(ADC_MEM1);
        //adcResult = ((adcResult*3.3)/16384);
        //adcResult1 = ((adcResult1*3.3)/16384);
        printf("\n adc result : %d\n", adcResult);
      printf("\n adc result1 : %d\n", adcResult1);

        MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig3);

                               if (direction == 1)
                               {
                                   b = b - 10;
                                   pwmConfig3.dutyCycle = b;
                                   if(b == 40) {
                                       direction = 0;
                                       //loop = 1;
                                   }

                                   for(a=700; a>0; a--);
                               }
                               else
                               {

                                   pwmConfig3.dutyCycle = b;
                                   b = b + 10;
                                   if(b == 170) {
                                       direction = 1;
                                       //loop = 0;
                                   }

                                   for(a=700; a>0; a--);

                               }

                           //}

                        //while (loop == 0 )
                          //     {
                             MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

                                       if (direction_1 == 1)
                                       {
                                           c = c - 10;
                                           pwmConfig.dutyCycle = c;
                                           if(c == 40) {
                                               direction_1 = 0;
                                               loop = 0;
                                           }

                                           for(a=700; a>0; a--);
                                       }
                                       else
                                       {

                                           pwmConfig.dutyCycle = c;
                                           c = c + 10;
                                           if(c == 170) {
                                               direction_1 = 1;
                                               loop = 1;
                                           }

                                           for(a=700; a>0; a--);

                                       }
    }
}
