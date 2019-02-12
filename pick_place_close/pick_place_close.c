//#include "driverlib.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include "main.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// START USER INPUT
// Proportional gain.
float kp = 10000;
// END USER INPUT

// Variable initializations
volatile uint16_t resultsBuffer[8],results[];
volatile int i,a,b,c,d,direction,direction_1,direction_2,loop;
volatile int range1 = 52 , range2 = 8, pwmval =1 ;
volatile int val1 , val2 ,val3;
volatile float desvolt1=0.7 , desvolt2 =1.4, desvolt3;
volatile float pidoutput1=0 , pidoutput2=0 , pidoutput3=0 ;
volatile float value1, value2 , value3 , error1 = 0, error2 = 0 , error3=0;
float control = 0;

// Sets the flag to control the rate at which the ADC measurements are taken.
volatile uint8_t flag = 0;

// Run controller at 4 MHz.
float dt = 0.250;

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


// Configures timer in Up mode.
const Timer_A_UpModeConfig upConfig_0 = {
    // Tie Timer A to SMCLK
    TIMER_A_CLOCKSOURCE_SMCLK,
    // Increment counter every 64 clock cycles: 1 MHz/64 = 15625 Hz
    TIMER_A_CLOCKSOURCE_DIVIDER_64,
    // Period of Timer A, generates interrupt every time we get to 15625
    1562,
    // Disable Timer A rollover interrupt
    TIMER_A_TAIE_INTERRUPT_DISABLE,
    // Enable Capture Compare interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
    //Clear counter upon initialization
    TIMER_A_DO_CLEAR
};

int main(void) {
    WDT_A_holdTimer();

    // Initializes variable to store result of getPIDoutput() function.
    float control = 0;

    /* Zero-filling buffer */
        memset(resultsBuffer, 0x00, 8 * sizeof(uint16_t));

        /* Setting reference voltage to 2.5  and enabling reference */
            MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
            MAP_REF_A_enableReferenceVoltage();

    //----------------CLOCK----------------//
    // Set DCO to 1 MHz.
    MAP_CS_setDCOFrequency(1E+6);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    //----------------OUTPUT PINS----------------//
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
               GPIO_PIN5 | GPIO_PIN4 | GPIO_PIN3 | GPIO_PIN2 | GPIO_PIN1
                       | GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
               GPIO_PIN7 | GPIO_PIN6, GPIO_TERTIARY_MODULE_FUNCTION);

    // Configuring GPIO 5.6 as peripheral output for PWM
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configure P2.4 for Timer_A0 Timer_A0.CCI0A_Out0
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configure P5.7 for Timer_A2 Timer_A2.CCI2A_Out2
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

          // Configuring Timer_A to have a period of approximately 20ms
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig2);


    //----------------PWM----------------//

    // SMCLK, Up Mode, Clear TACLR
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;

    //----------------ADC14----------------//
    // Enable ADC Module
    MAP_ADC14_enableModule();
    // Set P5.5 (A0) to be ADC functional.

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION);
    // Set to 10 bit resolution.
    MAP_ADC14_setResolution(ADC_10BIT);
    // Tie to SMCLK with no divider.
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
             0);
    /* Setting MCLK to REFO at 128Khz for LF mode setting SMCLK to 64Khz */
   MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
   MAP_CS_initClockSignal(CS_MCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
   MAP_CS_initClockSignal(CS_SMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_2);
   MAP_PCM_setPowerState(PCM_AM_LF_VCORE0);

    // Enables Conversion
    // Sets ENC bit.
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM7,true);
        MAP_ADC14_configureConversionMemory(ADC_MEM0,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A0, false);

        MAP_ADC14_configureConversionMemory(ADC_MEM2,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A2, false);

        MAP_ADC14_configureConversionMemory(ADC_MEM5,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A5, false);

        MAP_ADC14_enableInterrupt(ADC_INT7);

            /* Enabling Interrupts */
        MAP_Interrupt_enableInterrupt(INT_ADC14);
        MAP_Interrupt_enableMaster();

            // Configures sample timer; triggers each conversion manually.
        MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    MAP_ADC14_enableConversion();
    // Toggles conversion trigger to start the conversion.
    MAP_ADC14_toggleConversionTrigger();
    //----------------TIMER----------------//
            // Configures Timer A1.
   MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig_0);
            // Enables Timer A interrupt.
   MAP_Interrupt_enableInterrupt(INT_TA1_0);
            // Starts Timer A1
   MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    b = 80;
    c = 40;
    d = 40;
    direction = 0;
    direction_1 = 0;
    direction_2 = 0;

    while(1) {
     //   printf("check flag");
        // Flag is set to 1 every dt sec.
        if (flag==1) {
           // printf("flag =1");
            //If there are no conversions in progress.
            if (!MAP_ADC14_isBusy()) {

                // Get ADC conversion result.
            //    ADCResult = MAP_ADC14_getResult(ADC_MEM0);
              //  outputTemp = ADCResult * (3.3/1023)/0.01;

                // Toggles conversion trigger to start the conversion.
                MAP_ADC14_toggleConversionTrigger();


                for (i=0; i< range1 ; i++)
                {
                        MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);   // pin 2.4 - Lift servo
                        if (direction_1 == 1)
                                {
                                    c = c - pwmval;
                                    pwmConfig.dutyCycle = c;
                                    if(c == 40)
                                    {
                                        direction_1 = 0;
                                        loop = 0;
                                    }

                                    for(a=700; a>0; a--);
                                }
                                else
                                {

                                    pwmConfig.dutyCycle = c;
                                    c = c + pwmval;
                                    if(c == 170)
                                    {
                                        direction_1 = 1;
                                        loop = 1;
                                    }

                                    for(a=700; a>0; a--);

                                }
                        }

                       for (i=0; i< range2 ; i++)
                       {
                       MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig); //  pin 5.6 - Tilt servo
                               if (direction == 1)
                               {
                                   b = b - pwmval;
                                   pwmConfig.dutyCycle = b;
                                   if(b == 40)
                                   {
                                       direction = 0;
                                       loop = 1;
                                   }

                                   for(a=700; a>0; a--);
                               }
                               else
                               {
                                   pwmConfig.dutyCycle = b;
                                   b = b + pwmval;
                                   if(b == 170)
                                   {
                                       direction = 1;
                                       loop = 0;
                                   }

                                   for(a=700; a>0; a--);

                               }
                             }
                           for (i=0; i< range1 ; i++)
                           {
                           MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig2);      // pin 5.7 -Gripper Servo

                                  if (direction_2 == 1)
                                  {
                                      d = d - pwmval;
                                      pwmConfig2.dutyCycle = d;
                                      if(d == 40)
                                      {
                                          direction_2 = 0;
                                          //loop = 1;
                                      }

                                      for(a=700; a>0; a--);
                                  }
                                  else
                                  {

                                      pwmConfig2.dutyCycle = d;
                                      d = d + pwmval;
                                      if(d == 170)
                                      {
                                          direction_2 = 1;
                                         // loop = 0;
                                      }

                                      for(a=700; a>0; a--);

                                  }
                               }
                                  for (i=0; i< range2 ; i++)
                                    {
                                        MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig); //  pin 5.6 - tilt servo
                                                 if (direction == 1)
                                                 {
                                                     pwmConfig.dutyCycle = b;
                                                     b = b + pwmval;
                                                     if(b == 170)
                                                     {
                                                     direction = 0;
                                                     loop = 0;
                                                     }
                                                     for(a=700; a>0; a--);
                                                 }
                                                 else
                                                 {
                                                     pwmConfig.dutyCycle = b;
                                                     b = b - pwmval;
                                                     if(b == 40)
                                                     {
                                                       direction = 1;
                                                       loop = 1;
                                                     }

                                                     for(a=700; a>0; a--);

                                                 }
                                            }
                                  for (i=0; i< range1 ; i++)
                                  {
                                  MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);   // pin 2.4 - Lift servo

                                         if (direction_1 == 1)
                                             {
                                             pwmConfig.dutyCycle = c;
                                             c = c + pwmval;
                                             if(c == 170) {
                                              direction_1 = 0;
                                              loop = 1;
                                              }
                                                     for(a=700; a>0; a--);
                                                 }
                                                 else
                                                 {
                                                     c = c - pwmval;
                                                     pwmConfig.dutyCycle = c;
                                                     if(c == 40)
                                                     {
                                                         direction_1 = 1;
                                                         loop = 0;
                                                     }

                                                     for(a=700; a>0; a--);

                                                 }
                                         }
                // Sets the flag to zero.
                flag = 0;
            }
        }
    }
}

//void timerA_isr()
void TA1_0_IRQHandler(void)
{
    //printf( "\n I am here \n "  );
    // Clears interrupt.
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    MAP_ADC14_getMultiSequenceResult(resultsBuffer);

                                                   val1 = resultsBuffer[0]; // Obtain result of A0
                                                   val2 = resultsBuffer[2]; // Obtain result of A2
                                                   val3 = resultsBuffer[5]; // Obtain result of A5

                                             value1 = ((val1*2.5)/16384); // Normalise the values of A0, A2 and A5
                                             value2 = ((val2*2.5)/16384);
                                             value3 = ((val3*2.5)/16384);

                                             // Gets the PID control output which is used to set the duty cycle.
                                              getPIDOutput();
    // Set flag to run control cycle.
    flag = 1;
}

float getPIDOutput()
{
    // Initializes errors.
    float error = 0;
    float error_dot = 0;

    // Compute error on scale normalized by total voltage .
    error1 = (desvolt1 - value1)/2.5;
    error2 = (desvolt2 - value2)/2.5;
    error3 = (desvolt3 - value3)/2.5;

    // Compute PID control.
    pidoutput1 = (kp*error1);
    pidoutput2 = (kp*error2);
    pidoutput3 = (kp*error3);

   // printf("PID = %.2f\n", pidoutput1);

    // Set duty cycle based on control input.
   TA0CCR1 = pidoutput1;
   TA2CCR1 = pidoutput2;
   TA2CCR2 = pidoutput3;

}
