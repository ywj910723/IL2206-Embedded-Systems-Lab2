#include <stdio.h>
#include "bsp.h"

/*************************************************************/

/**
 * @brief Main function.
 * 
 * @return int 
 */
int main()
{
    BSP_Init();             /* Initialize all components on the lab-kit. */
    
    while (true) {
        /* 3 s: red */
        BSP_SetLED(LED_RED, true);
        BSP_SetLED(LED_YELLOW, false);
        BSP_SetLED(LED_GREEN, false);
        sleep_ms(3000);

        /* 1 s: red + yellow */
        BSP_SetLED(LED_RED, true);
        BSP_SetLED(LED_YELLOW, true);
        BSP_SetLED(LED_GREEN, false);
        sleep_ms(1000);

        /* 3 s: green */
        BSP_SetLED(LED_RED, false);
        BSP_SetLED(LED_YELLOW, false);
        BSP_SetLED(LED_GREEN, true);
        sleep_ms(3000);

        /* 1 s: yellow */
        BSP_SetLED(LED_RED, false);
        BSP_SetLED(LED_YELLOW, true);
        BSP_SetLED(LED_GREEN, false);
        sleep_ms(1000);
    }
}
/*-----------------------------------------------------------*/
