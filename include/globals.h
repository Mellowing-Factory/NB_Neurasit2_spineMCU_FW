#ifndef _GLOBALS_H
#define _GLOBALS_H

// release version---------------------
#define RELEASE_MODE
#define DEV_MODE
#define TEST_MODE

//-------------------------------------

// debug log level---------------------
#define LOG_LEVEL 4 //ERROR, WARN, INFO, DEBUG, (5) VERBOSE
#define CORE_DEBUG_LEVEL LOG_LEVEL
//-------------------------------

// version config---------------------
#define REV_CODE 0
#define FV_CODE 0
#define STAGE_CODE "DEV"
//-------------------------------

// model config---------------------
#define MODEL_NUMBER "NB_FSR_TEST_SLAVE"
//------------------------------- 

/*
   MEMORY KEEPING
*/
#define MEM_LOW 2048 // [Bytes] low memory threshold triggering a send cycle

/*
   GPIO pin map
*/
#define LED_PIN_GR     uint8_t(41)   // Green LED
#define LED_PIN_BL     uint8_t(42)  // Blue LED

// SDA_PIN, SCL_PIN, SLAVE_RX, SLAVE_TX
#define SDA2_PIN 12
#define SCL2_PIN 11

/*
    Timer Configurations 
*/
#define TIMER_DIVIDER_MICRO 80 // timer prescaler (MHz) (ESP32 default 80MHz). set timer ticker to 1MHz (1us)
#define TIMER_0     uint8_t(0) // Central timer to collect and send data

/*
   Irq Bit Flag
*/
#define CDS_IRQ                 BIT0
#define SENDCYCLE_IRQ           BIT3
#define SEND_SENSOR_IRQ         BIT4
#define CYCLIC_IRQ              BIT5
#define RESET_MANU              BIT6
#define UPDATE_REPORT_FILE_IRQ  BIT8
#define SEND_REPORT_FILE_IRQ    BIT9

/*
   Wait Bit Flag
*/
#define NOTIFY_BIT BIT0
#define DISCONNECTED_BIT BIT1
#define SEND_BIT BIT2

/*
   RTOS TASK CONF
*/
// Core 0: 
#define CORE_0 0
// Core 1: 
#define CORE_1 1

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE CORE_0
#else
#define ARDUINO_RUNNING_CORE CORE_1
#endif

/* CORE 0 Tasks */

#define TIMER_TASK_PRI 1
#define TIMER_TASK_STACK 4096
#define TIMER_TASK_CORE CORE_0

#define IRQ_TASK_PRI 1
#define IRQ_TASK_STACK 2048
#define IRQ_TASK_CORE CORE_0

/* CORE 1 Tasks */

#define UART_TASK_PRI 1
#define UART_TASK_STACK 4096
#define UART_TASK_CORE CORE_1

#define SENSORS_TASK_PRI 5
#define SENSORS_TASK_STACK 8192
#define SENSORS_TASK_CORE CORE_1

/* Central timer and data buffer */
#define TIMER_HZ 10

typedef struct {
    float pitch0 = 0;
    float pitch1 = 0;
    float pitch2 = 0;
    float pitch3 = 0;    
} All_data_t;

#include <Arduino.h>
#include <Ticker.h>
#include <set>
#include <array>
#include <algorithm>
#include <ArduinoJson.h>
#include <ESP32Time.h>

extern TaskHandle_t TaskIrq;
extern TaskHandle_t TaskData_h;
extern TaskHandle_t TaskUartSlave_h;
extern TaskHandle_t TaskUartMaster_h;
extern TaskHandle_t TaskTimer_h;
extern TaskHandle_t TaskSensors_h;

#endif