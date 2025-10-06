#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "bsp.h"

/* ---- single shared location ---- */
static volatile int32_t sharedAddress = 0;

/* ---- semaphores: A->B data ready, B->A processed ---- */
static SemaphoreHandle_t semA2B;   // A notifies B
static SemaphoreHandle_t semB2A;   // B notifies A

/* optional handles */
static TaskHandle_t tA = NULL, tB = NULL;

/* Task A: send 1,2,3...; read back -1,-2,-3... and print */
static void vTaskA(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(500);      // 500 ms pace
    TickType_t last = xTaskGetTickCount();             // baseline for DelayUntil
    int32_t n = 0;

    for (;;)
    {
        n += 1;
        sharedAddress = n;
        printf("Sending   : %ld\r\n", (long)n);

        xSemaphoreGive(semA2B);                        // tell B: data ready
        xSemaphoreTake(semB2A, portMAX_DELAY);         // wait until B processed

        printf("Receiving : %ld\r\n", (long)sharedAddress);

        vTaskDelayUntil(&last, period);                // stable timing
    }
}

/* Task B: read, multiply by -1, write back */
static void vTaskB(void *arg)
{
    (void)arg;
    for (;;)
    {
        xSemaphoreTake(semA2B, portMAX_DELAY);         // wait data from A
        sharedAddress = -sharedAddress;                // process in place
        xSemaphoreGive(semB2A);                        // tell A: done
    }
}

int main(void)
{
    BSP_Init();                                        // board init (UART/USB)

    semA2B = xSemaphoreCreateBinary();
    semB2A = xSemaphoreCreateBinary();
    configASSERT(semA2B && semB2A);

    /* A has printf -> give 512 stack for safety */
    configASSERT(xTaskCreate(vTaskA, "TaskA", 512, NULL, 2, &tA) == pdPASS);
    configASSERT(xTaskCreate(vTaskB, "TaskB", 512, NULL, 1, &tB) == pdPASS);

    vTaskStartScheduler();

    for (;;);                                          // should not reach here
}
