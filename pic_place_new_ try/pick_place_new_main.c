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
 * MSP432 ADC14 - Multiple Channel Sample without Repeat
 *
 * Description: In this code example, the feature of being able to scan multiple
 * ADC channels is demonstrated by the user a the DriverLib APIs.  Conversion
 * memory registers ADC_MEM0 - ADC_MEM7 are configured to read conversion
 * results from A0-A7 respectively. Conversion is enabled and then sampling is
 * toggled using a software toggle. Repeat mode is not enabled and sampling only
 * occurs once (and it is expected that the user pauses the debugger to observe
 * the results). Once the final sample has been taken, the interrupt for
 * ADC_MEM7 is triggered and the result is stored in the resultsBuffer buffer.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P5.5  |<--- A0 (Analog Input)
 *            |            P5.4  |<--- A1 (Analog Input)
 *            |            P5.3  |<--- A2 (Analog Input)
 *            |            P5.2  |<--- A3 (Analog Input)
 *            |            P5.1  |<--- A4 (Analog Input)
 *            |            P5.0  |<--- A5 (Analog Input)
 *            |            P4.7  |<--- A6 (Analog Input)
 *            |            P4.6  |<--- A7 (Analog Input)
 *            |                  |
 *            |                  |
 *
 ******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>

#include <string.h>

#include <stdbool.h>

static uint16_t resultsBuffer[8],results[];
int i,a,b,c,d,direction,direction_1,direction_2,loop, val1 , val2 ,val3;

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

extern Timer_A_PWMConfig pwmConfig2 =
{
TIMER_A_CLOCKSOURCE_SMCLK,
TIMER_A_CLOCKSOURCE_DIVIDER_1,
1300,
TIMER_A_CAPTURECOMPARE_REGISTER_2,
TIMER_A_OUTPUTMODE_RESET_SET,
10
};

int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_enableSleepOnIsrExit();

    /* Zero-filling buffer */
    memset(resultsBuffer, 0x00, 8 * sizeof(uint16_t));

    //![Simple REF Example]
    /* Setting reference voltage to 2.5  and enabling reference */
    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    MAP_REF_A_enableReferenceVoltage();
    //![Simple REF Example]

    /* Initializing ADC (MCLK/1/1) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
            0);

    /* Configuring GPIOs for Analog In */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
            GPIO_PIN5 | GPIO_PIN4 | GPIO_PIN3 | GPIO_PIN2 | GPIO_PIN1
                    | GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
            GPIO_PIN7 | GPIO_PIN6, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Setting MCLK to REFO at 128Khz for LF mode setting SMCLK to 64Khz */
       MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
       MAP_CS_initClockSignal(CS_MCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
       MAP_CS_initClockSignal(CS_SMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_2);
       MAP_PCM_setPowerState(PCM_AM_LF_VCORE0);

       // Configuring GPIO 5.6 as peripheral output for PWM
       MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

       // Configure P2.4 for Timer_A0 Timer_A0.CCI0A_Out0
       //
       MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

       // Configure P5.7 for Timer_A2 Timer_A2.CCI2A_Out2
       //
       MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

       // Configuring Timer_A to have a period of approximately 20ms
       MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig);
       MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
       MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig2);

       b = 40;
           c = 40;
           d = 40;
           direction = 0;
           direction_1 = 0;
           direction_2 = 0;
           loop = 0;

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM7 (A0 - A7)  with no repeat)
     * with internal 2.5v reference */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM7,true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
            ADC_INPUT_A0, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
            ADC_INPUT_A1, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM2,
            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
            ADC_INPUT_A2, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM3,
            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
            ADC_INPUT_A3, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM4,
            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
            ADC_INPUT_A4, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM5,
            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
            ADC_INPUT_A5, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM6,
            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
            ADC_INPUT_A6, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM7,
            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
            ADC_INPUT_A7, false);

    /* Enabling the interrupt when a conversion on channel 7 (end of sequence)
     *  is complete and enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT7);

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    /* Going to sleep */
    while (1)
    {
        MAP_PCM_gotoLPM0();
    }
}

/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM7. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    if(status & ADC_INT7)
    {
        MAP_ADC14_getMultiSequenceResult(resultsBuffer);

                                                 val1 = resultsBuffer[0];
                                                 val2 = resultsBuffer[2];
                                                 val3 = resultsBuffer[5];

                                         //  = ((resultsBuffer[i]*3.3)/16384);
                                           printf( "\n adc result of 0 = %d \n " ,resultsBuffer[0] );
                                           printf( "\n adc result of 1 = %d \n " ,resultsBuffer[2] );
                                           printf( "\n adc result of 2 = %d \n " ,resultsBuffer[5] );

                                           for (i=0; i< 13 ; i++)
                                                  {
        MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);   // pin 2.4

        if (direction_1 == 1)
                {
                    c = c - 5;
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
                    c = c + 5;
                    if(c == 170) {
                        direction_1 = 1;
                        loop = 1;
                    }

                    for(a=700; a>0; a--);

                }
        }

                                           for (i=0; i< 9 ; i++)
                                           {
       MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig); //  pin 5.6

               if (direction == 1)
               {
                   b = b - 5;
                   pwmConfig.dutyCycle = b;
                   if(b == 40) {
                       direction = 0;
                       loop = 1;
                   }

                   for(a=700; a>0; a--);
               }
               else
               {
                   pwmConfig.dutyCycle = b;
                   b = b + 5;
                   if(b == 170) {
                       direction = 1;
                       loop = 0;
                   }

                   for(a=700; a>0; a--);

               }
                                           }
                                           for (i=0; i< 13 ; i++)
                                                                                    {

           MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig2);      // pin 5.7

                  if (direction_2 == 1)
                  {
                      d = d - 5;
                      pwmConfig2.dutyCycle = d;
                      if(d == 40) {
                          direction_2 = 0;
                          //loop = 1;
                      }

                      for(a=700; a>0; a--);
                  }
                  else
                  {

                      pwmConfig2.dutyCycle = d;
                      d = d + 5;
                      if(d == 170) {
                          direction_2 = 1;
                         // loop = 0;
                      }

                      for(a=700; a>0; a--);

                  }
                                                                                    }
                 // b = 80;
                  for (i=0; i< 9 ; i++)
                                                             {
                        MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig); //  pin 5.6

                                 if (direction == 1)
                                 {
                                     pwmConfig.dutyCycle = b;
                                                                         b = b + 5;
                                                                         if(b == 170) {
                                                                             direction = 0;
                                                                             loop = 0;
                                     }

                                     for(a=700; a>0; a--);
                                 }
                                 else
                                 {
                                     pwmConfig.dutyCycle = b;
                                     b = b - 5;
                                                                          //pwmConfig.dutyCycle = b;
                                                                          if(b == 40) {
                                                                              direction = 1;
                                                                              loop = 1;

                                     }

                                     for(a=700; a>0; a--);

                                 }
                                                             }
                 // c= 80;
                  for (i=0; i< 13 ; i++)
                  {
                  MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);   // pin 2.4

                         if (direction_1 == 1)
                                 {
                             pwmConfig.dutyCycle = c;
                                                                 c = c + 5;
                                                                 if(c == 170) {
                                                                     direction_1 = 0;
                                                                     loop = 1;

                                     }

                                     for(a=700; a>0; a--);
                                 }
                                 else
                                 {
                                     c = c - 5;
                                                                         pwmConfig.dutyCycle = c;
                                                                         if(c == 40) {
                                                                             direction_1 = 1;
                                                                             loop = 0;


                                     }

                                     for(a=700; a>0; a--);

                                 }
                         }

    }

}


