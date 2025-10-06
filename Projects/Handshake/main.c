#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "bsp.h"

/* Task handles */
static TaskHandle_t redTaskHandle  = NULL;
static TaskHandle_t greenTaskHandle = NULL;

/* Two binary semaphores: start signal to Green, and ack back to Red */
static SemaphoreHandle_t semStartGreen = NULL;  // "Start this step"
static SemaphoreHandle_t semAckGreen   = NULL;  // "Green set, acknowledged"

/* Shared step index 0..3 (written by Red, read by Green) */
static volatile uint32_t g_step = 0;

/* Four-step pattern: (red, green) for each 2-second step */
typedef struct { bool red; bool green; } Step;
static const Step kSequence[4] = {
    { true,  true  },  // Step 0: RED ON,  GREEN ON
    { true,  false },  // Step 1: RED ON,  GREEN OFF
    { false, false },  // Step 2: RED OFF, GREEN OFF
    { false, true  }   // Step 3: RED OFF, GREEN ON
};

/* ---------------- Green task ----------------
   Waits for "start" from Red, sets GREEN LED accordingly, then acks back. */
static void vTaskGreen(void *args)
{
    (void)args;
    for (;;)
    {
        /* Wait for Red’s start signal */
        xSemaphoreTake(semStartGreen, portMAX_DELAY);

        /* Read the current step and set GREEN LED immediately */
        uint32_t s = g_step & 0x3;  // 0..3
        BSP_SetLED(LED_GREEN, kSequence[s].green);

        /* Acknowledge back to Red */
        xSemaphoreGive(semAckGreen);
    }
}

/* ---------------- Red task ----------------
   Drives the 2-second period, updates RED LED after Green has confirmed. */
static void vTaskRed(void *args)
{
    (void)args;
    const TickType_t periodTicks = pdMS_TO_TICKS(2000);  // 2 seconds per step
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t step = 0;

    for (;;)
    {
        /* Publish the step and tell Green to update */
        g_step = step;
        xSemaphoreGive(semStartGreen);

        /* Wait until Green confirms it has updated */
        xSemaphoreTake(semAckGreen, portMAX_DELAY);

        /* Now update RED LED for this step */
        BSP_SetLED(LED_RED, kSequence[step].red);

        /* Wait until the next 2-second boundary */
        vTaskDelayUntil(&xLastWakeTime, periodTicks);

        /* Advance to next step (0..3) */
        step = (step + 1u) & 0x3u;
    }
}

/* ---------------- Main ---------------- */
int main(void)
{
    BSP_Init();

    /* As required: start with both RED and GREEN ON */
    BSP_SetLED(LED_RED,   true);
    BSP_SetLED(LED_GREEN, true);

    /* Create the two binary semaphores (start empty) */
    semStartGreen = xSemaphoreCreateBinary();
    semAckGreen   = xSemaphoreCreateBinary();

    /* Create tasks: Red should have slightly higher priority to drive timing */
    xTaskCreate(vTaskGreen, "GreenTask", 512, NULL, 1, &greenTaskHandle);
    xTaskCreate(vTaskRed,   "RedTask",   512, NULL, 2, &redTaskHandle);

    /* Start the scheduler */
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;);
}
