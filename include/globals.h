/* Developer Note
(cshin)
- Test Code: Code to test out new code (to be committed or deleted)
- Test Temp: Temporary code just for testing (comment or delete prior to commit)
- Test Silenced: Existing code that was silenced for testing (uncomment or delete prior to commit)

(cshin)
- LEDs for V1.0.2 (pre-batch) hardware has Gr/Bl flipped, FIXED PINMAP (20230919)
*/

#ifndef _GLOBALS_H
#define _GLOBALS_H

// release version---------------------
#define RELEASE_MODE
#define DEV_MODE
#define TEST_MODE

// #define PRINT_SENSOR_DATA
#define TINY_ML_PREDICTION
// #define BLE_TEST
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
#define MODEL_NUMBER "WF-VALIDATION-BOARD"
#define BLE_ADVERTISE_NAME "WF-VALIDATION"
//------------------------------- 

// #define ROW_WISE_FLATTENING
// // #define PRINT_PREDICTION_DATA
// #define ADJUST_TINYML_PREDICTION

#define ACCEL


/*
   MEMORY KEEPING
*/
#define MEM_LOW 2048 // [Bytes] low memory threshold triggering a send cycle

/*
   GPIO pin map
*/
#define LED_PIN_GR     uint8_t(41)   // Green LED
// #define LED_PIN_BL     uint8_t(42)  // Blue LED
#define MOSI_PIN 5
#define MISO_PIN 3
#define SCLK_PIN 6
#define CS_PIN   9
#define INT1_PIN gpio_num_t(10)

#define SDA2_PIN 12
#define SCL2_PIN 11

/*
    Timer Configurations 
*/
#define TIMER_DIVIDER_MICRO 80 // timer prescaler (MHz) (ESP32 default 80MHz). set timer ticker to 1MHz (1us)
#define TIMER_0     uint8_t(0) // Central timer to collect and send data
#define TIMER_1     uint8_t(1)
#define TIMER_2     uint8_t(2)
#define TIMER_3     uint8_t(3)

// BCG Timer
#define GLOBAL_TIMER_NUM    TIMER_0
#define DATA_SET_TIMER_NUM  TIMER_1

/*
   Ticker Time (SECs)
*/
#define SEND_DATA_TIME      60
#define UPDATE_REPORT_TIME  10 * 60 // 10 minutes
#define INIT_REPORT_TIME    60 * 60 * 24 * 7 // 1 week
#define HOME_KEEPING_TIME   25

/*
   Application retry or thres time
*/
#define BLE_SERVER_TIME 120                                // sec
#define WIFI_RECYCLE_TIME 50                               // count. 200ms delay, 50 count -> 10s
#define SEND_REPORT_DATA_INTERVAL_TIME 24 * 60 * 60 * 1000 // 24 hours(ms)

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
#define TIMER_TASK_STACK 20000
#define TIMER_TASK_CORE CORE_0

#define IRQ_TASK_PRI 1
#define IRQ_TASK_STACK 2048
#define IRQ_TASK_CORE CORE_0

#define BLE_TASK_PRI 1
#define BLE_TASK_STACK 4096
#define BLE_TASK_CORE CORE_0

/* CORE 1 Tasks */

#define UART_TASK_PRI 1
#define UART_TASK_STACK 4096
#define UART_TASK_CORE CORE_1

#define SENSORS_TASK_PRI 5
#define SENSORS_TASK_STACK 8192
#define SENSORS_TASK_CORE CORE_1


/* Central timer and data buffer */
#define TIMER_HZ                10
#define TIMER2_HZ               20
#define DATA_BUFFER_CYCLE_TIME  1 // 0.1
#define NUM_SENSORS             2 // 20
#define DATA_BUFFER_LEN         (TIMER_HZ * DATA_BUFFER_CYCLE_TIME * NUM_SENSORS * 2)

// ADS BUFFERS
#define ADS_SAMPLING_HZ 10
#define ADS_BUFFER_LEN  ADS_SAMPLING_HZ*60

#define BLE_DATA_MAXIMUM_SIZE 500

#define MAC_MAXIMUM_SIZE 30
#define BLE_MANU_MAXIMUM_SIZE 20
#define IDV_MAXIMUM_SIZE 8
#define D_ID_MAXIMUM_SIZE MAC_MAXIMUM_SIZE + IDV_MAXIMUM_SIZE

#define COMMAND_MAXIMUM_SIZE 100
#define UUID_MAXIMUM_SIZE 40
#define ADD_DATA_MAXIMUM_SIZE 512

#define THING_NAME_MAXIMUM_SIZE 128
#define CERT_MAXIMUM_SIZE 4096
#define TOPIC_MAXIMUM_SIZE 128
#define VERSION_MAXIMUM_SIZE 10
#define MAXIMUM_BASE64_DEVICE_DATA_SIZE 25
#define MAXIMUM_UUID_STR_SIZE 40

#define MAXIMUM_CONFIG_DATA_SIZE 512
#define MAXIMUM_DEVICE_FILE_DATA_SIZE 4096
#define MAXIMUM_REPORT_FILE_DATA_SIZE 8192
#define MAXIMUM_CERT_FILE_DATA_SIZE 6000

#define SEND_QUEUE_SIZE 20 // deimum number of messages in payload send queue [1 = no queue]

#define UTC 9
#define NTP_INTERVAL 60000 * 5
#define CHECK_LOOP_STUCK_THRES 90000
#define CHECK_MQTT_STUCK_THRES 30000

#define ALLOCATION(T, SIZE) psramFound() ? (T *)ps_malloc((SIZE) * sizeof(T)) : (T *)malloc((SIZE) * sizeof(T))
#define ALLOCATION_OBJ(T) psramFound() ? ps_malloc(sizeof(T)) : malloc(sizeof(T))
#define ROUND_DATA(DATA) roundf(DATA * 100) / 100
#define INT_8_TO_INT_16(d1, d2) ((uint16_t)d1 << 8) | d2

#define D_NUM 8

#include <Arduino.h>

enum sendprio_t {
    prio_low,
    prio_normal,
    prio_high
};

enum msgtype_t {
    ble_client,
    ble_client_server,
    ble_scan,
    mqtt_publish
};

enum reset_reason_t {
    rst_undefined,                      // 0
    rst_ble_server_timeout,             // 1
    rst_ble_server_wifi_set_complete,   // 2
    rst_device_reset_manu,              // 3
    rst_failed_to_init_spiffs,          // 4
    rst_memory_low,                     // 5
    rst_main_ble_client_stuck,          // 6
    rst_mqtt_stuck,                     // 7
    rst_mqtt_wifi_disconnected,         // 8
    rst_restart_manually,               // 9
    rst_wifi_set_manually,              // 10
    rst_ota_process_finish,             // 11
    rst_failed_to_init_sht,             // 12
    rst_failed_to_connect_mqtt,         // 13
    rst_main_ble_scan_stuck,            // 14
    rst_mqtt_abort,                     // 15
    rst_cannot_get_cert,                // 16
    rst_user_wake,                      // 17
    rst_manufacture_test,               // 18
    rst_periodic,                       // 19
    rst_ota                             // 20
};

enum report_category_t {
    mqtt_check_pub_try,
    mqtt_check_pub_success,
    mqtt_check_pub_retry,
    mqtt_check_pub_fail,
    mqtt_check_disconnect,

    ble_check_attempt,
    ble_check_attempt_success,
    ble_check_attempt_fail,
    ble_check_try,

    ble_fail_connect,
    ble_fail_disconnected,
    ble_fail_get_service,
    ble_fail_notif_find,
    ble_fail_notif,
    ble_fail_write_find,
    ble_fail_write_cannot,
    ble_fail_write_value,
    ble_fail_write_invalid,

    ble_scan_attempt,
    ble_scan_find,
    ble_scan_cannot_find_all,
};

#define MAX_REPORT_ARRAY 100

typedef struct {
    char root_ca[CERT_MAXIMUM_SIZE];

    char thing_name[THING_NAME_MAXIMUM_SIZE];
    char version[VERSION_MAXIMUM_SIZE];

    int bootCnt = 0;
    reset_reason_t current_reset_reason;
    reset_reason_t prev_reset_reason;
} configData_t; 

typedef struct {
    sendprio_t MessagePrio;
    msgtype_t MessageType;
    uint8_t index;
} Queue_message_t;

typedef struct {
    uint8_t buffer[ADS_BUFFER_LEN];
    int16_t index;
} Ads_message_t;

typedef struct {
    uint8_t buffer[DATA_BUFFER_LEN];
    int16_t index;
} Data_message_t;

// typedef struct {
//     int16_t accel[3];
//     int16_t gyro[3];
// } IMU_data;

// typedef struct {
// 	int16_t accel0[3];
// 	int16_t gyro0[3];
// 	int16_t accel1[3];
// 	int16_t gyro1[3];
// 	int16_t accel2[3];
// 	int16_t gyro2[3];

// 	int16_t ads1;
// 	int16_t ads2;
// } slave_ESP32_data;


// WIRE1 is on left, WIRE2 is on right
#define BUFFER_LEN 30
#define TOF_THRESHOLD 175

typedef struct {
    
#ifdef ACCEL
	// int16_t M_accel0[3]; // upper spine
	int16_t M_accel1[3]; // left strap front
	int16_t M_accel2[3]; // right strap front
	int16_t M_accel3[3]; // left strap back
	int16_t M_accel4[3]; // right strap back

    int16_t S_accel0[3]; // lower spine
	int16_t S_accel1[3]; // left leg strap front
	int16_t S_accel2[3]; // right leg strap front
#endif

    // int16_t M_gyro0[3];
	int16_t M_gyro1[3];
	int16_t M_gyro2[3];
	int16_t M_gyro3[3];
	int16_t M_gyro4[3];

	int16_t M_ads1; // left strap front
	int16_t M_ads2; // right strap front
	int16_t M_ads3; // left strap back
	int16_t M_ads4; // right strap back

    int16_t M_tof;

	int16_t S_gyro0[3];
	int16_t S_gyro1[3];
	int16_t S_gyro2[3];

	int16_t S_ads1; // left leg strap front
	int16_t S_ads2; // right leg strap front

    int16_t S_ads3_1; // lambda left FSR
    // int16_t S_ads3_2; // lambda left strain gauge
    int16_t S_ads3_3; // lambda right FSR
    // int16_t S_ads3_4; // lambda right strain gauge
    
} All_data_t;


class PredictionDataClass {
    public:

        void resetBuffers(void);
        uint16_t addData(All_data_t allData);

        int16_t ADS1_buffer[BUFFER_LEN];
        int16_t IMU1_x_buffer[BUFFER_LEN];
        int16_t IMU1_y_buffer[BUFFER_LEN];
        int16_t IMU1_z_buffer[BUFFER_LEN];

        int16_t ADS3_buffer[BUFFER_LEN];
        int16_t IMU3_x_buffer[BUFFER_LEN];
        int16_t IMU3_y_buffer[BUFFER_LEN];
        int16_t IMU3_z_buffer[BUFFER_LEN];

        int16_t TOF_buffer[BUFFER_LEN];
        int16_t TOF_1sec_min;

        int16_t dummy[BUFFER_LEN];
        
        uint16_t index = 0;
    
};

// typedef struct {
//     int16_t ADS3_buffer[BUFFER_LEN];
//     int16_t IMU3_x_buffer[BUFFER_LEN];
//     int16_t IMU3_y_buffer[BUFFER_LEN];
//     int16_t IMU3_z_buffer[BUFFER_LEN];

//     int16_t ADS1_buffer[BUFFER_LEN];
//     int16_t IMU1_x_buffer[BUFFER_LEN];
//     int16_t IMU1_y_buffer[BUFFER_LEN];
//     int16_t IMU1_z_buffer[BUFFER_LEN];

//     int16_t TOF_buffer[BUFFER_LEN];

//     int16_t index = 0;
// } PredictionDataClass;


#ifdef ACCEL
    #define SLAVE_MESSAGE_LENGTH 50
    #define NUM_SLAVE_SENSORS 22
#else
    #define SLAVE_MESSAGE_LENGTH 32
    #define NUM_SLAVE_SENSORS 13
#endif

void initAllSensors();

#include <Ticker.h>
#include <set>
#include <array>
#include <algorithm>
#include <ArduinoJson.h>
#include <ESP32Time.h>

extern configData_t *cfg;

extern QueueHandle_t taskQueue;

extern TaskHandle_t irqHandlerTask; // sendData, Button
extern TaskHandle_t TaskData_h;

extern TaskHandle_t TaskUartMaster_h;
extern TaskHandle_t TaskTimer_h;
extern TaskHandle_t TaskTimer2_h;
extern TaskHandle_t TaskBle_h;
extern TaskHandle_t TaskSensors_h;

#endif