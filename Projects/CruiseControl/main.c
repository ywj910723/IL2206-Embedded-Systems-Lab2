/**
 * @file main.c
 * @brief Part 2: Cruise Control (Button, Control Law, Vehicle, Display)
 * @version 1.1
 * @date 2025-09-30
 */

 #include <stdio.h>
 #include "FreeRTOS.h"
 #include "task.h"
 #include "queue.h"
 #include "semphr.h"
 #include "bsp.h"
 #include "hardware/clocks.h"
 
 /* ---------------- Configuration ---------------- */
 
 #define GAS_STEP 2      /* Throttle increment when gas pedal is pressed */
 
 #define P 100           /* Very simple PID gains */
 #define I 1
 #define D 5
 
 #define PERIOD_BUTTON_MS   50
 #define PERIOD_VEHICLE_MS  100
 #define PERIOD_CONTROL_MS  200
 #define PERIOD_DISPLAY_MS  500
 
 /* ---------------- Task handles ---------------- */
 
 static TaskHandle_t xButton_handle;
 static TaskHandle_t xControl_handle;
 static TaskHandle_t xVehicle_handle;
 static TaskHandle_t xDisplay_handle;
 
 /* ---------------- Queues (size=1) ---------------- */
 
 static QueueHandle_t xQueueVelocity;        /* uint16_t (0.1 m/s units) */
 static QueueHandle_t xQueueTargetVelocity;  /* uint16_t (0.1 m/s units) */
 static QueueHandle_t xQueuePosition;        /* uint16_t (meters*10, 0..24000) */
 static QueueHandle_t xQueueThrottle;        /* uint16_t (0..80) */
 static QueueHandle_t xQueueCruiseControl;   /* bool */
 static QueueHandle_t xQueueGasPedal;        /* bool */
 static QueueHandle_t xQueueBrakePedal;      /* bool */
 
 /* ---------------- Provided (do not modify) ---------------- */
 
 uint16_t adjust_position(uint16_t position, int16_t velocity,
                          int8_t acceleration, uint16_t time_interval)
 {
     int16_t new_position = position + velocity * time_interval / 1000
                          + acceleration / 2 * (time_interval / 1000) * (time_interval / 1000);
 
     if (new_position > 24000) {
         new_position -= 24000;
     } else if (new_position < 0){
         new_position += 24000;
     }
 
     return new_position;
 }
 
 int16_t adjust_velocity(int16_t velocity, int8_t acceleration,
                         bool brake_pedal, uint16_t time_interval)
 {
     int16_t new_velocity;
     uint8_t brake_retardation = 50;
 
     if (brake_pedal == false) {
         new_velocity = velocity + (float)((acceleration * time_interval) / 1000);
         if (new_velocity <= 0) {
             new_velocity = 0;
         }
     } else {
         if ((float)(brake_retardation * time_interval) / 1000 > velocity) {
             new_velocity = 0;
         } else {
             new_velocity = velocity - (float)brake_retardation * time_interval / 1000;
         }
     }
 
     return new_velocity;
 }
 
 /* ---------------- Utilities ---------------- */
 
 static inline void write_position(uint16_t position)
 {
     /* Map 0..24000 to 24 LEDs (100 m per LED) */
     uint32_t led_reg;
     int idx = position / 1000;        /* 0..23 */
     if (idx > 23) idx = 23;
     led_reg = (1u << idx);
     BSP_ShiftRegWriteAll((uint8_t*)&led_reg);
 }
 
 /* ultra-simple PID (as per assignment’s “very simple” guidance) */
 static uint16_t calc_throttle_with_PID(uint16_t target_velocity, uint16_t velocity)
 {
     static int32_t integral = 0;
     static int16_t prev_error = 0;
 
     int16_t error = (int16_t)target_velocity - (int16_t)velocity;
     integral += error;
     int16_t derivative = error - prev_error;
     prev_error = error;
 
     int32_t output = P * error + I * integral + D * derivative;
     if (output < 0)  output = 0;
     if (output > 80) output = 80;
     return (uint16_t)output;
 }
 
 /* ---------------- Tasks ---------------- */
 
 /* Button: reads SW5/SW6/SW7, owns GREEN/RED LEDs, manages cruise request */
 static void vButtonTask(void *args)
 {
     const TickType_t xPeriod = (TickType_t)args;            /* ticks */
     TickType_t xLastWakeTime = xTaskGetTickCount();
 
     bool lastCruiseSw = BSP_GetInput(SW_6);                 /* high when not pressed */
     bool cruiseEnabled = false;
 
     for (;;) {
         bool gas   = !BSP_GetInput(SW_7);                   /* active low */
         bool brake = !BSP_GetInput(SW_5);                   /* active low */
         bool sw6   =  BSP_GetInput(SW_6);
 
         /* read current velocity (0.1 m/s units) */
         uint16_t vel = 0;
         (void)xQueuePeek(xQueueVelocity, &vel, 0);
 
         /* Falling edge on SW6 requests cruise; enable only if conditions are met */
         if (lastCruiseSw != sw6 && sw6 == false) {
             if (!gas && !brake && vel >= 250) {             /* >= 25 m/s */
                 cruiseEnabled = true;
                 (void)xQueueOverwrite(xQueueTargetVelocity, &vel);
             } else {
                 cruiseEnabled = false;
             }
         }
         lastCruiseSw = sw6;
 
         /* Gas or Brake immediately disables cruise (spec) */
         if (gas || brake) {
             cruiseEnabled = false;
         }
 
         /* Publish inputs */
         (void)xQueueOverwrite(xQueueGasPedal,      &gas);
         (void)xQueueOverwrite(xQueueBrakePedal,    &brake);
         (void)xQueueOverwrite(xQueueCruiseControl, &cruiseEnabled);
 
         /* LEDs owned by Button for Gas/Brake */
         BSP_SetLED(LED_GREEN, gas);
         BSP_SetLED(LED_RED,   brake);
 
         vTaskDelayUntil(&xLastWakeTime, xPeriod);
     }
 }
 
 /* Control Law: computes throttle, owns YELLOW LED (cruise status) */
 static void vControlTask(void *args)
 {
     const TickType_t xPeriod = (TickType_t)args;            /* ticks */
     TickType_t xLastWakeTime = xTaskGetTickCount();
 
     uint16_t throttle = 0;
 
     for (;;) {
         bool cruise = false, gas = false, brake = false;
         uint16_t velocity = 0, target_velocity = 0;
 
         (void)xQueuePeek(xQueueCruiseControl, &cruise, 0);
         (void)xQueuePeek(xQueueGasPedal,      &gas,    0);
         (void)xQueuePeek(xQueueBrakePedal,    &brake,  0);
         (void)xQueuePeek(xQueueVelocity,      &velocity, 0);
         (void)xQueuePeek(xQueueTargetVelocity,&target_velocity, 0);
 
         if (brake) {
             throttle = 0;
             cruise = false;
         } else if (gas) {
             cruise = false;
             uint16_t t = throttle + GAS_STEP;
             throttle = (t > 80) ? 80 : t;
         } else if (cruise && velocity >= 250) {             /* keep only when >= 25 m/s */
             throttle = calc_throttle_with_PID(target_velocity, velocity);
         } else {
             throttle = 0;
         }
 
         (void)xQueueOverwrite(xQueueThrottle, &throttle);
 
         /* Yellow LED owned by Control Law */
         BSP_SetLED(LED_YELLOW, cruise);
 
         vTaskDelayUntil(&xLastWakeTime, xPeriod);
     }
 }
 
 /* Vehicle: updates position/velocity; behavior unchanged (signed types + dt) */
 static void vVehicleTask(void *args)
 {
     const TickType_t xPeriod = (TickType_t)args;            /* ticks */
     TickType_t xLastWakeTime = xTaskGetTickCount();
 
     uint16_t position = 0;                                  /* 0..24000 (0..2400 m) */
     uint16_t velocity = 0;                                  /* -200..700 in 0.1 m/s */
     uint16_t throttle = 0;
     bool brake_pedal = false;
 
     for (;;) {
         (void)xQueuePeek(xQueueThrottle,   &throttle,    0);
         (void)xQueuePeek(xQueueBrakePedal, &brake_pedal, 0);
 
         int16_t wind_factor;                                /* -10..20 */
         int8_t  retardation;                                /*  20..-10 */
         int8_t  acceleration;                               /*  40..-20 */
 
         if ((int16_t)velocity > 0)
             wind_factor = (int16_t)(velocity * velocity / 10000) + 1;
         else
             wind_factor = (int16_t)((-1) * velocity * velocity / 10000) + 1;
 
         if (position < 4000)
             retardation = (int8_t)wind_factor;              /* flat */
         else if (position < 8000)
             retardation = (int8_t)(wind_factor + 8);        /* uphill */
         else if (position < 12000)
             retardation = (int8_t)(wind_factor + 16);       /* steep uphill */
         else if (position < 16000)
             retardation = (int8_t)wind_factor;              /* flat */
         else if (position < 20000)
             retardation = (int8_t)(wind_factor - 8);        /* downhill */
         else
             retardation = (int8_t)(wind_factor - 16);       /* steep downhill */
 
         acceleration = (int8_t)(throttle / 2) - retardation;
 
         const uint16_t dt_ms = (uint16_t)pdTICKS_TO_MS(xPeriod);
         position = adjust_position(position, (int16_t)velocity, acceleration, dt_ms);
         velocity = adjust_velocity((int16_t)velocity, acceleration, brake_pedal, dt_ms);
 
         (void)xQueueOverwrite(xQueueVelocity, &velocity);
         (void)xQueueOverwrite(xQueuePosition, &position);
 
         vTaskDelayUntil(&xLastWakeTime, xPeriod);
     }
 }
 
 /* Display: shows throttle & velocity on 7-seg (zero-padded), position on 24 LEDs */
 static void vDisplayTask(void *args)
 {
     const TickType_t xPeriod = (TickType_t)args;            /* ticks */
     TickType_t xLastWakeTime = xTaskGetTickCount();
 
     BSP_7SegClear();
     char dsp[9];
 
     for (;;) {
         uint16_t velocity = 0, position = 0, throttle = 0;
 
         (void)xQueuePeek(xQueueVelocity, &velocity, 0);
         (void)xQueuePeek(xQueuePosition, &position, 0);
         (void)xQueuePeek(xQueueThrottle, &throttle, 0);
 
         /* Zero-padded to avoid blank spaces (e.g., show "0000" at startup) */
         snprintf(dsp, sizeof(dsp), "%02u%02u", throttle, (uint16_t)(velocity / 10U));
         BSP_7SegDispString(dsp);
 
         write_position(position);
 
         vTaskDelayUntil(&xLastWakeTime, xPeriod);
     }
 }
 
 /* ---------------- Main ---------------- */
 
 int main(void)
 {
     BSP_Init(); /* Ensure single-core FreeRTOS in FreeRTOSConfig.h as per assignment */
 
     /* Create queues before tasks (size=1 each) */
     xQueueCruiseControl   = xQueueCreate(1, sizeof(bool));
     xQueueGasPedal        = xQueueCreate(1, sizeof(bool));
     xQueueBrakePedal      = xQueueCreate(1, sizeof(bool));
     xQueueVelocity        = xQueueCreate(1, sizeof(uint16_t));
     xQueueTargetVelocity  = xQueueCreate(1, sizeof(uint16_t));
     xQueuePosition        = xQueueCreate(1, sizeof(uint16_t));
     xQueueThrottle        = xQueueCreate(1, sizeof(uint16_t));
 
     /* Seed initial values to avoid empty peeks on first cycle */
     bool bfalse = false;
     uint16_t u0 = 0;
     (void)xQueueOverwrite(xQueueCruiseControl,  &bfalse);
     (void)xQueueOverwrite(xQueueGasPedal,       &bfalse);
     (void)xQueueOverwrite(xQueueBrakePedal,     &bfalse);
     (void)xQueueOverwrite(xQueueVelocity,       &u0);
     (void)xQueueOverwrite(xQueueTargetVelocity, &u0);
     (void)xQueueOverwrite(xQueuePosition,       &u0);
     (void)xQueueOverwrite(xQueueThrottle,       &u0);
 
     /* Create periodic tasks. RMS: shorter period -> higher priority */
     #define T(ms) ((void*)pdMS_TO_TICKS(ms))
 
     xTaskCreate(vButtonTask,  "Button",  512, T(PERIOD_BUTTON_MS),  5, &xButton_handle);   /* 50 ms  */
     xTaskCreate(vVehicleTask, "Vehicle", 512, T(PERIOD_VEHICLE_MS), 4, &xVehicle_handle);  /* 100 ms */
     xTaskCreate(vControlTask, "Control", 512, T(PERIOD_CONTROL_MS), 3, &xControl_handle);  /* 200 ms */
     xTaskCreate(vDisplayTask, "Display", 512, T(PERIOD_DISPLAY_MS), 2, &xDisplay_handle);  /* 500 ms */
 
     vTaskStartScheduler();
 
     for(;;) { /* should never return */ }
     return 0;
 }
 