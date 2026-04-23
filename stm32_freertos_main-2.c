/**
 * @file    main.c
 * @brief   STM32 FreeRTOS Multi-Task Firmware System
 * @target  STM32F411RE (Nucleo-64)
 * @clock   16 MHz HSI
 *
 * Architecture:
 *   - vSensorTask  (Priority 3) : Triggers ADC, waits for semaphore from ISR
 *   - vUARTTask    (Priority 2) : Dequeues sensor data, transmits over UART2
 *   - vLogTask     (Priority 1) : Periodic data logging (flash/SD stub)
 *
 * Inter-task sync:
 *   - xSensorQueue    : ADC results passed from ISR → UART task
 *   - xSensorSemaphore: Binary semaphore — ISR signals Sensor task
 */

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>

/* ─── Handles ─────────────────────────────────────────────────── */
static QueueHandle_t     xSensorQueue;
static SemaphoreHandle_t xSensorSemaphore;

static TaskHandle_t xSensorTaskHandle;
static TaskHandle_t xUARTTaskHandle;
static TaskHandle_t xLogTaskHandle;

/* ─── Data Types ───────────────────────────────────────────────── */
typedef struct {
    uint16_t raw;       /* 12-bit ADC raw value  */
    uint32_t tick;      /* FreeRTOS tick at capture */
} SensorData_t;

/* ─── ADC Init (Register Level) ───────────────────────────────── */
static void ADC_Init(void)
{
    /* Enable clocks: GPIOA, ADC1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* PA0 → Analog mode (MODER[1:0] = 11) */
    GPIOA->MODER |= (3U << (0 * 2));

    /* ADC config: 12-bit resolution, right-align, single conversion */
    ADC1->CR1   = 0;
    ADC1->CR2   = 0;
    ADC1->SQR1  = 0;                   /* sequence length = 1 */
    ADC1->SQR3  = 0;                   /* CH0 first in sequence */
    ADC1->SMPR2 |= (7U << (0 * 3));   /* 480 cycles — max sampling time */

    /* Enable EOC interrupt, then power on ADC */
    ADC1->CR1 |= ADC_CR1_EOCIE;
    ADC1->CR2 |= ADC_CR2_ADON;

    /* NVIC: priority below configMAX_SYSCALL_INTERRUPT_PRIORITY (5) */
    NVIC_SetPriority(ADC_IRQn, 6);
    NVIC_EnableIRQ(ADC_IRQn);
}

/* ─── UART2 Init (Register Level, 9600-8N1 @ 16 MHz) ──────────── */
static void UART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 = TX, PA3 = RX → AF7 */
    GPIOA->MODER  &= ~((3U << (2*2)) | (3U << (3*2)));
    GPIOA->MODER  |=  ((2U << (2*2)) | (2U << (3*2)));
    GPIOA->AFR[0] |=  ((7U << (2*4)) | (7U << (3*4)));

    /* BRR = fPCLK1 / baud = 16,000,000 / 9600 ≈ 1667 (0x0683) */
    USART2->BRR = 0x0683;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* ─── UART2 Helpers ────────────────────────────────────────────── */
static void UART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (uint8_t)c;
}

static void UART2_SendStr(const char *s)
{
    while (*s) UART2_SendChar(*s++);
}

/* Minimal uint→string, no stdlib needed */
static void uint_to_str(uint32_t val, char *buf, uint8_t *len)
{
    char tmp[10];
    uint8_t i = 0;
    if (val == 0) { buf[0] = '0'; buf[1] = '\0'; *len = 1; return; }
    while (val) { tmp[i++] = '0' + (val % 10); val /= 10; }
    *len = i;
    for (uint8_t j = 0; j < i; j++) buf[j] = tmp[i - 1 - j];
    buf[i] = '\0';
}

/* ─── ADC IRQ Handler ──────────────────────────────────────────── */
void ADC_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (ADC1->SR & ADC_SR_EOC) {
        SensorData_t data = {
            .raw  = (uint16_t)(ADC1->DR & 0x0FFF),
            .tick = xTaskGetTickCountFromISR()
        };

        /* Send to queue — non-blocking from ISR */
        xQueueSendFromISR(xSensorQueue, &data, &xHigherPriorityTaskWoken);

        /* Signal sensor task */
        xSemaphoreGiveFromISR(xSensorSemaphore, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ─── Task: Sensor (Priority 3 — Highest) ─────────────────────── */
static void vSensorTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        /* Trigger software-start ADC conversion */
        ADC1->CR2 |= ADC_CR2_SWSTART;

        /* Block until ISR gives semaphore (conversion done) */
        xSemaphoreTake(xSensorSemaphore, portMAX_DELAY);

        /* 100 ms between conversions */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ─── Task: UART Communication (Priority 2) ───────────────────── */
static void vUARTTask(void *pvParameters)
{
    (void)pvParameters;
    SensorData_t rxData;
    char buf[48];
    uint8_t len;

    for (;;) {
        /* Block until data is available in queue */
        if (xQueueReceive(xSensorQueue, &rxData, portMAX_DELAY) == pdTRUE) {
            /*
             * Convert 12-bit ADC → voltage (mV) with 3.3 V reference
             * voltage_mV = raw * 3300 / 4095
             */
            uint32_t voltage_mv = ((uint32_t)rxData.raw * 3300U) / 4095U;

            /* Build output string: "ADC:XXXX V:YYYY mV T:ZZZZ\r\n" */
            uint8_t pos = 0;

            memcpy(buf + pos, "ADC:", 4); pos += 4;
            uint_to_str(rxData.raw, buf + pos, &len); pos += len;

            memcpy(buf + pos, " V:", 3); pos += 3;
            uint_to_str(voltage_mv, buf + pos, &len); pos += len;

            memcpy(buf + pos, "mV T:", 5); pos += 5;
            uint_to_str(rxData.tick, buf + pos, &len); pos += len;

            memcpy(buf + pos, "\r\n", 3); pos += 3; /* +3 includes '\0' */

            UART2_SendStr(buf);
        }
    }
}

/* ─── Task: Data Logging (Priority 1 — Lowest) ────────────────── */
static void vLogTask(void *pvParameters)
{
    (void)pvParameters;
    SensorData_t logData;
    uint32_t logCount = 0;
    char logBuf[32];
    uint8_t len;

    for (;;) {
        /*
         * Peek without removing — logging observes, UART task consumes.
         * In a production system, replace UART log with flash write
         * (e.g., SPI flash or SDIO using DMA).
         */
        if (xQueuePeek(xSensorQueue, &logData, pdMS_TO_TICKS(500)) == pdTRUE) {
            logCount++;

            uint8_t pos = 0;
            memcpy(logBuf + pos, "LOG#", 4); pos += 4;
            uint_to_str(logCount, logBuf + pos, &len); pos += len;
            memcpy(logBuf + pos, " RAW:", 5); pos += 5;
            uint_to_str(logData.raw, logBuf + pos, &len); pos += len;
            memcpy(logBuf + pos, "\r\n", 3);

            UART2_SendStr(logBuf);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ─── Main ─────────────────────────────────────────────────────── */
int main(void)
{
    ADC_Init();
    UART2_Init();

    /* Create queue: 10 items of SensorData_t */
    xSensorQueue    = xQueueCreate(10, sizeof(SensorData_t));
    xSensorSemaphore = xSemaphoreCreateBinary();

    configASSERT(xSensorQueue     != NULL);
    configASSERT(xSensorSemaphore != NULL);

    /* Create tasks */
    xTaskCreate(vSensorTask, "Sensor", 128, NULL, 3, &xSensorTaskHandle);
    xTaskCreate(vUARTTask,   "UART",   256, NULL, 2, &xUARTTaskHandle);
    xTaskCreate(vLogTask,    "Log",    128, NULL, 1, &xLogTaskHandle);

    /* Start FreeRTOS scheduler — never returns */
    vTaskStartScheduler();

    /* Execution should never reach here */
    for (;;);
}
