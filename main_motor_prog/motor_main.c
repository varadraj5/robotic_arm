/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>

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

// PWM - PIN - 5.6 - TIMER_A2_BASE
// PWM - PIN - 2.4 - TIMER_A0_BASE

int main(void)
{
    /* Halting the watchdog */
    MAP_WDT_A_holdTimer();

    int a,b,c,direction,direction_1,direction_2,loop;

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
    direction = 0;
    direction_1 = 0;
    direction_2 = 0;
    loop = 0;
while (1)
   {
    /* Sleeping when not in use */
   //while (loop==1)
    //{
        MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig);

        if (direction == 1)
        {
            b = b - 5;
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
            b = b + 5;
            if(b == 170) {
                direction = 1;
                loop = 0;
            }

            for(a=700; a>0; a--);

        }

    //}

 //while (loop == 0 )
      {
        MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

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
//
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig2);

           if (direction == 1)
           {
               b = b - 5;
               pwmConfig2.dutyCycle = b;
               if(b == 40) {
                   direction = 0;
                   //loop = 1;
               }

               for(a=700; a>0; a--);
           }
           else
           {

               pwmConfig2.dutyCycle = b;
               b = b + 5;
               if(b == 170) {
                   direction = 1;
                  // loop = 0;
               }

               for(a=700; a>0; a--);

           }
    }
}
