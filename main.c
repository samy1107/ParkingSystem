#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Port F RGB pin masks */
#define LED_RED             (1U << 1)   /* PF1 */
#define LED_BLUE            (1U << 2)   /* PF2 */
#define LED_GREEN           (1U << 3)   /* PF3 */
#define LED_MASK            (LED_RED | LED_BLUE | LED_GREEN)

/* Button pin masks */
#define BTN_DRIVER_OPEN     (1U << 0)   /* PF0 - Driver Open button, active low */
#define BTN_OBSTACLE        (1U << 4)   /* PF4 - Obstacle button, active low */

#define BTN_DRIVER_CLOSE    (1U << 1)   /* PB1 - Driver Close button, active high */

#define BTN_SECURITY_OPEN   (1U << 0)   /* PE0 - Security Open button, active high */
#define BTN_SECURITY_CLOSE  (1U << 1)   /* PE1 - Security Close button, active high */

#define BTN_LIMIT_OPEN      (1U << 0)   /* PD0 - Open limit switch, active high */
#define BTN_LIMIT_CLOSE     (1U << 1)   /* PD1 - Close limit switch, active high */

/* Clock-gate masks: Port B, D, E, F */
#define RCGCGPIO_ALL        ((1U << 1) | (1U << 3) | (1U << 4) | (1U << 5))

/* System Constants */
#define AUTO_MODE_THRESHOLD_MS  400

typedef enum {
    IDLE_CLOSED,
    IDLE_OPEN,
    OPENING,
    CLOSING,
    STOPPED_MIDWAY,
    REVERSING
} GateState_t;

typedef enum {
    EV_OPEN_REQ,
    EV_CLOSE_REQ,
    EV_STOP_REQ,
    EV_LIMIT_OPEN_HIT,
    EV_LIMIT_CLOSE_HIT
} GateEvent_t;

/* Global RTOS Handles */
QueueHandle_t xGateQueue;
SemaphoreHandle_t xStateMutex;
GateState_t eCurrentState = IDLE_CLOSED;

/* Hardware Initialization */
static void GPIO_Init(void) {
    SYSCTL_RCGCGPIO_R |= RCGCGPIO_ALL;
    while ((SYSCTL_PRGPIO_R & RCGCGPIO_ALL) != RCGCGPIO_ALL) { }

    /*
       Port F:
       PF0 = Driver Open button input with pull-up
       PF4 = Obstacle button input with pull-up
       PF1, PF2, PF3 = RGB LED outputs

       Important:
       PF0 is locked by default on TM4C123, so we must unlock it.
    */
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R |= (BTN_DRIVER_OPEN | BTN_OBSTACLE | LED_MASK);

    GPIO_PORTF_DIR_R |= LED_MASK;
    GPIO_PORTF_DIR_R &= ~(BTN_DRIVER_OPEN | BTN_OBSTACLE);

    GPIO_PORTF_PUR_R |= (BTN_DRIVER_OPEN | BTN_OBSTACLE);

    GPIO_PORTF_DEN_R |= (BTN_DRIVER_OPEN | BTN_OBSTACLE | LED_MASK);

    /*
       Port B:
       PB1 = Driver Close button input with pull-down
    */
    GPIO_PORTB_DIR_R &= ~BTN_DRIVER_CLOSE;
    GPIO_PORTB_PDR_R |= BTN_DRIVER_CLOSE;
    GPIO_PORTB_DEN_R |= BTN_DRIVER_CLOSE;

    /*
       Port E:
       PE0 = Security Open button input with pull-down
       PE1 = Security Close button input with pull-down
    */
    GPIO_PORTE_DIR_R &= ~(BTN_SECURITY_OPEN | BTN_SECURITY_CLOSE);
    GPIO_PORTE_PDR_R |= (BTN_SECURITY_OPEN | BTN_SECURITY_CLOSE);
    GPIO_PORTE_DEN_R |= (BTN_SECURITY_OPEN | BTN_SECURITY_CLOSE);

    /*
       Port D:
       PD0 = Open limit switch input with pull-down
       PD1 = Close limit switch input with pull-down
    */
    GPIO_PORTD_DIR_R &= ~(BTN_LIMIT_OPEN | BTN_LIMIT_CLOSE);
    GPIO_PORTD_PDR_R |= (BTN_LIMIT_OPEN | BTN_LIMIT_CLOSE);
    GPIO_PORTD_DEN_R |= (BTN_LIMIT_OPEN | BTN_LIMIT_CLOSE);
}

/* Logic for Input Processing */
void vInputTask(void *pvParameters) {
    bool prevOpenReq = false;
    bool prevCloseReq = false;

    TickType_t openPressTime = 0;
    TickType_t closePressTime = 0;

    bool isManualOpen = false;
    bool isManualClose = false;

    for (;;) {
        /*
           Read Inputs

           PF0 and PF4 use pull-up:
           Not pressed = 1
           Pressed     = 0

           PB1, PE0, PE1, PD0, PD1 use pull-down:
           Not pressed = 0
           Pressed     = 1
        */

        bool dOpen = (GPIO_PORTF_DATA_R & BTN_DRIVER_OPEN) == 0;
        bool obstacle = (GPIO_PORTF_DATA_R & BTN_OBSTACLE) == 0;

        bool dClose = (GPIO_PORTB_DATA_R & BTN_DRIVER_CLOSE) != 0;

        bool sOpen = (GPIO_PORTE_DATA_R & BTN_SECURITY_OPEN) != 0;
        bool sClose = (GPIO_PORTE_DATA_R & BTN_SECURITY_CLOSE) != 0;

        bool limO = (GPIO_PORTD_DATA_R & BTN_LIMIT_OPEN) != 0;
        bool limC = (GPIO_PORTD_DATA_R & BTN_LIMIT_CLOSE) != 0;

        /*
           Priority and Conflict Logic

           Security buttons have priority.
           If security close is pressed, driver open is ignored.
           If security open is pressed, driver close is ignored.
        */
        bool curOpenReq = sOpen || (dOpen && !sClose);
        bool curCloseReq = sClose || (dClose && !sOpen);

        /*
           Simultaneous Open and Close:
           If open and close are requested at same time, stop the gate.
        */
        if (curOpenReq && curCloseReq) {
            GateEvent_t ev = EV_STOP_REQ;
            xQueueSend(xGateQueue, &ev, 0);
        } else {
            /*
               Open Button Logic

               Short press:
               - Sends OPEN request once.
               - Gate keeps opening automatically.

               Long press / hold:
               - After AUTO_MODE_THRESHOLD_MS, it becomes manual mode.
               - When released, gate stops.
            */
            if (curOpenReq && !prevOpenReq) {
                GateEvent_t ev = EV_OPEN_REQ;
                xQueueSend(xGateQueue, &ev, 0);

                openPressTime = xTaskGetTickCount();
                isManualOpen = false;
            }
            else if (curOpenReq && !isManualOpen) {
                if ((xTaskGetTickCount() - openPressTime) > pdMS_TO_TICKS(AUTO_MODE_THRESHOLD_MS)) {
                    isManualOpen = true;
                }
            }
            else if (!curOpenReq && prevOpenReq) {
                if (isManualOpen) {
                    GateEvent_t ev = EV_STOP_REQ;
                    xQueueSend(xGateQueue, &ev, 0);
                }
            }

            /*
               Close Button Logic

               Short press:
               - Sends CLOSE request once.
               - Gate keeps closing automatically.

               Long press / hold:
               - After AUTO_MODE_THRESHOLD_MS, it becomes manual mode.
               - When released, gate stops.
            */
            if (curCloseReq && !prevCloseReq) {
                GateEvent_t ev = EV_CLOSE_REQ;
                xQueueSend(xGateQueue, &ev, 0);

                closePressTime = xTaskGetTickCount();
                isManualClose = false;
            }
            else if (curCloseReq && !isManualClose) {
                if ((xTaskGetTickCount() - closePressTime) > pdMS_TO_TICKS(AUTO_MODE_THRESHOLD_MS)) {
                    isManualClose = true;
                }
            }
            else if (!curCloseReq && prevCloseReq) {
                if (isManualClose) {
                    GateEvent_t ev = EV_STOP_REQ;
                    xQueueSend(xGateQueue, &ev, 0);
                }
            }
        }

        /*
           Limit Switch Events
        */
        if (limO) {
            GateEvent_t ev = EV_LIMIT_OPEN_HIT;
            xQueueSend(xGateQueue, &ev, 0);
        }

        if (limC) {
            GateEvent_t ev = EV_LIMIT_CLOSE_HIT;
            xQueueSend(xGateQueue, &ev, 0);
        }

        prevOpenReq = curOpenReq;
        prevCloseReq = curCloseReq;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* Gate State Machine */
void vGateControlTask(void *pvParameters) {
    GateEvent_t event;

    for (;;) {
        if (xQueueReceive(xGateQueue, &event, portMAX_DELAY) == pdPASS) {
            xSemaphoreTake(xStateMutex, portMAX_DELAY);

            switch (event) {
                case EV_OPEN_REQ:
                    if (eCurrentState != IDLE_OPEN && eCurrentState != OPENING) {
                        eCurrentState = OPENING;
                    }
                    break;

                case EV_CLOSE_REQ:
                    if (eCurrentState != IDLE_CLOSED && eCurrentState != CLOSING) {
                        eCurrentState = CLOSING;
                    }
                    break;

                case EV_STOP_REQ:
                    if (eCurrentState == OPENING || eCurrentState == CLOSING || eCurrentState == REVERSING) {
                        eCurrentState = STOPPED_MIDWAY;
                    }
                    break;

                case EV_LIMIT_OPEN_HIT:
                    if (eCurrentState == OPENING || eCurrentState == REVERSING) {
                        eCurrentState = IDLE_OPEN;
                    }
                    break;

                case EV_LIMIT_CLOSE_HIT:
                    if (eCurrentState == CLOSING) {
                        eCurrentState = IDLE_CLOSED;
                    }
                    break;
            }

            xSemaphoreGive(xStateMutex);
        }
    }
}

/* Safety Task for Obstacle Detection */
void vSafetyTask(void *pvParameters) {
    for (;;) {
        /*
           PF4 uses pull-up:
           Pressed obstacle button = 0
        */
        if ((GPIO_PORTF_DATA_R & BTN_OBSTACLE) == 0) {
            xSemaphoreTake(xStateMutex, portMAX_DELAY);

            if (eCurrentState == CLOSING) {
                eCurrentState = STOPPED_MIDWAY;
                xSemaphoreGive(xStateMutex);

                vTaskDelay(pdMS_TO_TICKS(50));

                xSemaphoreTake(xStateMutex, portMAX_DELAY);
                eCurrentState = REVERSING;
                xSemaphoreGive(xStateMutex);

                vTaskDelay(pdMS_TO_TICKS(500));

                xSemaphoreTake(xStateMutex, portMAX_DELAY);
                eCurrentState = STOPPED_MIDWAY;
                xSemaphoreGive(xStateMutex);
            }
            else {
                xSemaphoreGive(xStateMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* LED Task */
void vLEDTask(void *pvParameters) {
    for (;;) {
        GateState_t state;

        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        state = eCurrentState;
        xSemaphoreGive(xStateMutex);

        uint32_t val = 0;

        if (state == OPENING || state == REVERSING) {
            val = LED_GREEN;
        }
        else if (state == CLOSING) {
            val = LED_RED;
        }
        else if (state == STOPPED_MIDWAY) {
            val = LED_BLUE;
        }
        else {
            val = 0;
        }

        GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~LED_MASK) | val;

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main(void) {
    GPIO_Init();

    xGateQueue = xQueueCreate(10, sizeof(GateEvent_t));
    xStateMutex = xSemaphoreCreateMutex();

    if (xGateQueue != NULL && xStateMutex != NULL) {
        xTaskCreate(vInputTask,       "Input",   128, NULL, 3, NULL);
        xTaskCreate(vGateControlTask, "Gate",    128, NULL, 2, NULL);
        xTaskCreate(vSafetyTask,      "Safety",  128, NULL, 4, NULL);
        xTaskCreate(vLEDTask,         "LED",     128, NULL, 2, NULL);

        vTaskStartScheduler();
    }

    for (;;);
}