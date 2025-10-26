/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/* ===========================================================================
 * INCLUDES
 * =========================================================================== */

/* Standard C Libraries */
#include <stdlib.h>

/* ChibiOS Core */
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

/* lwIP Network Stack */
#include "lwipthread.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/netif.h"

/* Debug and RTT */
#include "SEGGER_RTT_Channel.h"
#include "SEGGER_RTT.h"

/* USB Host */
#include "usbh/debug.h"
#if HAL_USBH_USE_HID
#include "usbh/dev/hid.h"
#endif

/* Hardware Drivers */
#include "nrf24l01.h"
#include "nrf24l01_interface.h"
#include "nrf24l01_basic.h"

/* Shell */
#include "shell.h"

/* FlashDB (disabled) */
// #include "fdb_port.h"
// #include "w25qxx_interface.h"

/* ===========================================================================
 * CONFIGURATION CONSTANTS AND MACROS
 * =========================================================================== */

/* Thread Monitoring Configuration */
#define MAX_MONITORED_THREADS       10
#define MONITOR_DISPLAY_INTERVAL_MS 5000  // Display every 5 seconds

/* Network Configuration */
#define UDP_SERVER_PORT             12345
#define UDP_BUFFER_SIZE             1024
#define JOY_STREAM_ENABLE           1
#define JOY_STREAM_PORT             12346

/* Shell Configuration */
#define SHELL_WA_SIZE               THD_WORKING_AREA_SIZE(1024)

/* Event Masks */
#define EVT_DS5_READY               EVENT_MASK(0)
#define EVT_NRF24_INTERRUPT         EVENT_MASK(1)

/* PS5 Controller Button Constants */
#define PS5_BTN_UP                  0x00
#define PS5_BTN_RIGHT               0x01
#define PS5_BTN_DOWN                0x02
#define PS5_BTN_LEFT                0x03
#define PS5_BTN_SQUARE              0x04
#define PS5_BTN_CROSS               0x05
#define PS5_BTN_CIRCLE              0x06
#define PS5_BTN_TRIANGLE            0x07
#define PS5_BTN_L1                  0x08
#define PS5_BTN_R1                  0x09
#define PS5_BTN_L2                  0x0A
#define PS5_BTN_R2                  0x0B
#define PS5_BTN_CREATE              0x0C
#define PS5_BTN_OPTIONS             0x0D
#define PS5_BTN_L3                  0x0E
#define PS5_BTN_R3                  0x0F
#define PS5_BTN_PS                  0x10
#define PS5_BTN_TOUCHPAD            0x11
#define PS5_BTN_MICROPHONE          0x12

/* ===========================================================================
 * TYPE DEFINITIONS AND STRUCTURES
 * =========================================================================== */

/* ---------------------------------------------------------------------------
 * Thread Monitoring Structures
 * --------------------------------------------------------------------------- */

typedef struct {
    const char* name;
    thread_t* thread_ptr;
    systime_t last_loop_time;
    systime_t current_loop_time;
    systime_t loop_duration;
    uint32_t loop_count;
    systime_t min_loop_time;
    systime_t max_loop_time;
    systime_t total_loop_time;
    systime_t first_loop_time;        // Time of first loop for frequency calculation
    systime_t last_wake_time;         // Time of last wake-up for frequency calculation
    uint32_t wake_count;              // Number of wake-ups
    bool is_active;
} thread_monitor_t;

/* ---------------------------------------------------------------------------
 * PS5 DualSense Controller Structures
 * --------------------------------------------------------------------------- */

#if HAL_USBH_USE_HID

/* DualSense D-Pad states */
typedef enum {
  DS5_DPAD_UP         = 0,
  DS5_DPAD_UP_RIGHT   = 1,
  DS5_DPAD_RIGHT      = 2,
  DS5_DPAD_DOWN_RIGHT = 3,
  DS5_DPAD_DOWN       = 4,
  DS5_DPAD_DOWN_LEFT  = 5,
  DS5_DPAD_LEFT       = 6,
  DS5_DPAD_UP_LEFT    = 7,
  DS5_DPAD_RELEASED   = 8
} ds5_dpad_t;

/* PS5 Button structure (24-bit packed) */
typedef union {
  struct {
        // Byte 0
    uint8_t dpad : 4;       /* ds5_dpad_t */
    uint8_t square : 1;
    uint8_t cross : 1;
    uint8_t circle : 1;
    uint8_t triangle : 1;

        // Byte 1
    uint8_t l1 : 1;
    uint8_t r1 : 1;
    uint8_t l2 : 1;         /* digital */
    uint8_t r2 : 1;         /* digital */
    uint8_t create : 1;     /* Share/Create */
        uint8_t options : 1;    /* Options/Menu */
    uint8_t l3 : 1;
    uint8_t r3 : 1;

        // Byte 2
    uint8_t ps : 1;         /* PS button */
    uint8_t touchpad : 1;   /* Touchpad press */
    uint8_t mic : 1;        /* Mic mute */
    uint8_t reserved : 5;
  } __attribute__((packed));
  uint32_t val : 24;
} __attribute__((packed)) PS5Buttons;

/* PS5 Touchpad data structure */
typedef struct {
  struct {
        uint8_t counter : 7;    /* Increments every time a finger is touching the touchpad */
        uint8_t touching : 1;   /* The top bit is cleared if the finger is touching the touchpad */
    uint16_t x : 12;        /* 0..1919 */
    uint16_t y : 12;        /* 0..1079 */
        } __attribute__((packed)) finger[2]; // 0 = first finger, 1 = second finger
} __attribute__((packed)) PS5TouchpadXY;

/* PS5 Status structure */
typedef union {
        struct {
        // First byte
                uint8_t headphone : 1;
        uint8_t dummy : 2;      /* Seems to change when a jack is plugged in */
        uint8_t usb : 1;        /* Charging */
                uint8_t dummy2: 4;

        // Second byte
                uint8_t mic : 1;
                uint8_t dummy3 : 3;
        } __attribute__((packed));
        uint16_t val;
} __attribute__((packed)) PS5Status;

/* Full PS5 Data structure - received from USB HID */
typedef struct {
        /* Button and joystick values */
    uint8_t hatValue[4];        // 0-3 bytes (Left stick X, Y, Right stick X, Y)
    uint8_t trigger[2];         // 4-5 (L2, R2 analog)

    uint8_t sequence_number;    // 6

    PS5Buttons btn;             // 7-9

    uint8_t reserved[5];        // 0xA-0xD

        /* Gyro and accelerometer values */
    int16_t gyroX, gyroZ, gyroY;    // 0x0F - 0x14
    int16_t accX, accZ, accY;       // 0x15-0x1A
    int32_t sensor_timestamp;       // 0x1B-0x1E

    uint8_t reserved2;              // 0x1F

    // 0x20 - 0x27 touchpad data
    PS5TouchpadXY xy;

#if 0 // The status byte depends on if it's sent via USB or Bluetooth, so is not parsed for now
    uint8_t reserved3;              // 0x28
    uint8_t rightTriggerFeedback;   // 0x29
    uint8_t leftTriggerFeedback;    // 0x2A
    uint8_t reserved4[10];          // 0x2B - 0x34
    PS5Status status;               // 0x35-0x36
#endif
} __attribute__((packed)) PS5DataFull;

/* Trimmed PS5 Data structure (31 bytes) - for nRF24L01 transmission
 * Size: hatValue[4] + trigger[2] + sequence_number[1] + btn[3] + reserved[5] + 
 *       gyroX,gyroZ,gyroY[6] + accX,accZ,accY[6] + sensor_timestamp[4] = 31 bytes */
typedef struct {
        /* Button and joystick values */
    uint8_t hatValue[4];        // 0-3 bytes
    uint8_t trigger[2];         // 4-5

    uint8_t sequence_number;    // 6

    PS5Buttons btn;             // 7-9

    uint8_t reserved[5];        // 0xA-0xD

        /* Gyro and accelerometer values */
    int16_t gyroX, gyroZ, gyroY;    // 0x0F - 0x14
    int16_t accX, accZ, accY;       // 0x15-0x1A
    int32_t sensor_timestamp;       // 0x1B-0x1E
} __attribute__((packed)) PS5Data;

#endif /* HAL_USBH_USE_HID */

/* ===========================================================================
 * GLOBAL VARIABLES
 * =========================================================================== */

/* ---------------------------------------------------------------------------
 * Thread Monitor Variables
 * Used by: ThreadMonitor
 * --------------------------------------------------------------------------- */
static thread_monitor_t thread_monitors[MAX_MONITORED_THREADS];
static uint8_t monitor_count = 0;

#if HAL_USBH_USE_HID
/* ---------------------------------------------------------------------------
 * USB HID and PS5 Data Variables
 * Used by: UsbHostThread, ThreadTestHID, and data consumers (UDP/nRF24)
 * --------------------------------------------------------------------------- */

/* PS5 Data Double Buffer - latest-only publication via event broadcast */
static PS5DataFull ds5Buf[2];
static volatile uint8_t ds5PublishedIndex = 0;
static volatile uint32_t ds5Generation = 0;
static event_source_t esDS5Ready;

/* USB HID Configuration */
static USBH_DEFINE_BUFFER(uint8_t report[HAL_USBHHID_MAX_INSTANCES][64]);
static USBHHIDConfig hidcfg[HAL_USBHHID_MAX_INSTANCES];
#endif

/* ---------------------------------------------------------------------------
 * nRF24L01 Variables
 * Used by: NRF24InterruptThread, NRF24TxThread
 * --------------------------------------------------------------------------- */
static event_source_t esNRF24Interrupt;
static event_listener_t nrf24_interrupt_listener;

/* ---------------------------------------------------------------------------
 * Network/UDP Variables
 * Used by: UdpServerThread
 * --------------------------------------------------------------------------- */
static char udp_dest_ip[16] = "192.168.0.10";  /* Default UDP destination IP */
static mutex_t udp_dest_mutex;                  /* Mutex for thread-safe UDP dest access */

/* ---------------------------------------------------------------------------
 * Shell Variables
 * Used by: Shell thread
 * --------------------------------------------------------------------------- */
static char shell_history[SHELL_MAX_HIST_BUFF];
static char *shell_completions[SHELL_MAX_COMPLETIONS];

/* ===========================================================================
 * FORWARD DECLARATIONS
 * =========================================================================== */

/* Thread Monitoring Functions */
static bool register_thread_monitor(const char* name, thread_t* thread_ptr);
static void update_thread_loop_start(const char* name);
static void update_thread_loop_end(const char* name);

/* Network Reconfiguration Function */
static bool network_reconfigure(const char* ip_str, const char* gateway_str, 
                                const char* netmask_str);

/* UDP Destination Configuration Functions */
static bool udp_set_destination(const char* dest_ip);
static void udp_get_destination(char* dest_ip, size_t buf_size);

/* ===========================================================================
 * HELPER FUNCTIONS AND CALLBACKS
 * =========================================================================== */

/* ---------------------------------------------------------------------------
 * Debug Print Interface Functions
 * --------------------------------------------------------------------------- */

void w25qxx_interface_debug_print(const char *const fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    chvprintf((BaseSequentialStream *)&RTT_S0, fmt, ap);
    va_end(ap);
}

void nrf24l01_interface_debug_print(const char *const fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    chvprintf((BaseSequentialStream *)&RTT_S0, fmt, ap);
    va_end(ap);
}

/* ---------------------------------------------------------------------------
 * PS5 Controller Helper Functions
 * --------------------------------------------------------------------------- */

#if HAL_USBH_USE_HID

/* Parse raw HID report buffer into PS5DataFull
 * Supports USB (report ID 0x01) and Bluetooth (report ID 0x31)
 * Returns true on success */
static bool ds5_from_hid_report(const uint8_t *buf, uint16_t len, PS5DataFull *out) {
  if (buf == NULL || out == NULL) return false;
  if (len < 2) return false;

  if (buf[0] == 0x01) {
    /* USB: payload starts at buf+1 */
    if (len < (uint16_t)(1 + sizeof(PS5DataFull))) return false;
    memcpy(out, buf + 1, sizeof(PS5DataFull));
    return true;
  }
  return false;
}

/* Convert PS5DataFull to trimmed PS5Data for nRF24L01 transmission */
static void convert_to_trimmed(const PS5DataFull *full, PS5Data *trimmed) {
    if (full == NULL || trimmed == NULL) return;
    
    // Copy the common fields (hatValue to sensor_timestamp)
    memcpy(trimmed, full, sizeof(PS5Data));
}

/* Print hex dump of PS5Data structure for debugging */
static void print_ps5data_hex(const PS5Data *data, const char *label) {
    if (data == NULL) return;
    
    chprintf((BaseSequentialStream *)&RTT_S0, "\n=== %s ===\n", label);
    chprintf((BaseSequentialStream *)&RTT_S0, "Size: %zu bytes\n", sizeof(PS5Data));
    
    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0; i < sizeof(PS5Data); i++) {
        if (i % 16 == 0) {
            chprintf((BaseSequentialStream *)&RTT_S0, "\n%04zX: ", i);
        }
        chprintf((BaseSequentialStream *)&RTT_S0, "%02X ", bytes[i]);
    }
    chprintf((BaseSequentialStream *)&RTT_S0, "\n");
    
    // Print field breakdown
    chprintf((BaseSequentialStream *)&RTT_S0, "\nField breakdown:\n");
    chprintf((BaseSequentialStream *)&RTT_S0, "hatValue[0-3]:     %02X %02X %02X %02X\n", 
             data->hatValue[0], data->hatValue[1], data->hatValue[2], data->hatValue[3]);
    chprintf((BaseSequentialStream *)&RTT_S0, "trigger[0-1]:      %02X %02X\n", 
             data->trigger[0], data->trigger[1]);
    chprintf((BaseSequentialStream *)&RTT_S0, "sequence_number:   %02X\n", data->sequence_number);
    chprintf((BaseSequentialStream *)&RTT_S0, "btn.val:           %06X\n", data->btn.val);
    chprintf((BaseSequentialStream *)&RTT_S0, "reserved[0-4]:     %02X %02X %02X %02X %02X\n", 
             data->reserved[0], data->reserved[1], data->reserved[2], data->reserved[3], data->reserved[4]);
    chprintf((BaseSequentialStream *)&RTT_S0, "gyroX,gyroZ,gyroY: %04X %04X %04X\n", 
             (uint16_t)data->gyroX, (uint16_t)data->gyroZ, (uint16_t)data->gyroY);
    chprintf((BaseSequentialStream *)&RTT_S0, "accX,accZ,accY:    %04X %04X %04X\n", 
             (uint16_t)data->accX, (uint16_t)data->accZ, (uint16_t)data->accY);
    chprintf((BaseSequentialStream *)&RTT_S0, "sensor_timestamp:  %08X\n", (uint32_t)data->sensor_timestamp);
    chprintf((BaseSequentialStream *)&RTT_S0, "==================\n\n");
}

#endif /* HAL_USBH_USE_HID */

/* ---------------------------------------------------------------------------
 * Thread Monitoring Functions
 * --------------------------------------------------------------------------- */

/* Register a thread for monitoring */
static bool register_thread_monitor(const char* name, thread_t* thread_ptr) {
    if (monitor_count >= MAX_MONITORED_THREADS) {
        return false;
    }
    
    thread_monitor_t* monitor = &thread_monitors[monitor_count];
    monitor->name = name;
    monitor->thread_ptr = thread_ptr;
    monitor->last_loop_time = chVTGetSystemTime();
    monitor->current_loop_time = monitor->last_loop_time;
    monitor->loop_duration = 0;
    monitor->loop_count = 0;
    monitor->min_loop_time = 0xFFFFFFFF;
    monitor->max_loop_time = 0;
    monitor->total_loop_time = 0;
    monitor->first_loop_time = 0;
    monitor->last_wake_time = 0;
    monitor->wake_count = 0;
    monitor->is_active = true;
    
    monitor_count++;
    return true;
}

/* Update thread loop timing - call this at the start of each thread loop */
static void update_thread_loop_start(const char* name) {
    systime_t now = chVTGetSystemTime();
    
    for (uint8_t i = 0; i < monitor_count; i++) {
        if (thread_monitors[i].is_active && 
            strcmp(thread_monitors[i].name, name) == 0) {
            thread_monitor_t* monitor = &thread_monitors[i];
            
            monitor->current_loop_time = now;
            
            // Track wake-up frequency
            monitor->wake_count++;
            if (monitor->first_loop_time == 0) {
                monitor->first_loop_time = now;
            }
            monitor->last_wake_time = now;
            
            break;
        }
    }
}

/* Update thread loop timing - call this at the end of each thread loop */
static void update_thread_loop_end(const char* name) {
    systime_t now = chVTGetSystemTime();
    
    for (uint8_t i = 0; i < monitor_count; i++) {
        if (thread_monitors[i].is_active && 
            strcmp(thread_monitors[i].name, name) == 0) {
            thread_monitor_t* monitor = &thread_monitors[i];
            
            // Calculate loop duration
            monitor->loop_duration = now - monitor->current_loop_time;
            
            // Update statistics
            monitor->loop_count++;
            monitor->total_loop_time += monitor->loop_duration;
            
            if (monitor->loop_duration < monitor->min_loop_time) {
                monitor->min_loop_time = monitor->loop_duration;
            }
            if (monitor->loop_duration > monitor->max_loop_time) {
                monitor->max_loop_time = monitor->loop_duration;
            }
            
            monitor->last_loop_time = now;
            break;
        }
    }
}

/* ---------------------------------------------------------------------------
 * nRF24L01 Interrupt Callback
 * --------------------------------------------------------------------------- */

static void interrupt_callback(uint8_t type, uint8_t num, uint8_t *buf, uint8_t len) {
    switch (type) {
        case NRF24L01_INTERRUPT_RX_DR: {
            uint8_t i;
            nrf24l01_interface_debug_print("nrf24l01: irq receive with pipe %d with %d.\n", num, len);
            for (i = 0; i < len; i++) {
                nrf24l01_interface_debug_print("0x%02X ", buf[i]);
            }
            nrf24l01_interface_debug_print(".\n");
            break;
        }
        case NRF24L01_INTERRUPT_TX_DS:
            // nrf24l01_interface_debug_print("nrf24l01: irq send ok.\n");
            break;
        case NRF24L01_INTERRUPT_MAX_RT:
            // nrf24l01_interface_debug_print("nrf24l01: irq reach max retry times.\n");
            break;
        case NRF24L01_INTERRUPT_TX_FULL:
            break;
        default:
            break;
    }
}

/* nRF24L01 GPIO interrupt callback (ISR context) */
void nrf24l01_interface_gpio_interrupt_callback(void *args) {
    (void)args;
    chSysLockFromISR();
    chEvtBroadcastI(&esNRF24Interrupt);
    chSysUnlockFromISR();
}

/* ---------------------------------------------------------------------------
 * Network Callbacks
 * --------------------------------------------------------------------------- */

void myLinkUpCallback(void *p) {
    struct netif *ifc = (struct netif*) p;
    chprintf((BaseSequentialStream *)&RTT_S0, 
             "Ethernet reconnected! IP: %d.%d.%d.%d\n",
             ip4_addr1(&ifc->ip_addr), ip4_addr2(&ifc->ip_addr),
             ip4_addr3(&ifc->ip_addr), ip4_addr4(&ifc->ip_addr));
}

void myLinkDownCallback(void *p) {
    (void)p;
    chprintf((BaseSequentialStream *)&RTT_S0, "Ethernet disconnected!\n");
}

/* Network Reconfiguration Function
 * 
 * Reconfigures the network interface at runtime using lwipReconfigure.
 * Always uses STATIC IP mode.
 * 
 * Parameters:
 *   ip_str:      IP address string (e.g., "192.168.0.100")
 *   gateway_str: Gateway address string (e.g., "192.168.0.1")
 *   netmask_str: Netmask string (e.g., "255.255.255.0")
 * 
 * Returns:
 *   true on success, false on failure (invalid IP format or NULL parameter)
 * 
 * Example usage:
 *   // Change to 192.168.1.200
 *   network_reconfigure("192.168.1.200", "192.168.1.1", "255.255.255.0");
 * 
 *   // Change to different subnet
 *   network_reconfigure("10.0.0.50", "10.0.0.1", "255.255.255.0");
 * 
 *   // Change to 192.168.0.150
 *   network_reconfigure("192.168.0.150", "192.168.0.1", "255.255.255.0");
 */
static bool network_reconfigure(const char* ip_str, const char* gateway_str, 
                                const char* netmask_str) {
    
    lwipreconf_opts_t reconf_opts;
    ip4_addr_t ip_addr, gateway_addr, netmask_addr;
    
    // Validate parameters
    if (ip_str == NULL || gateway_str == NULL || netmask_str == NULL) {
        chprintf((BaseSequentialStream *)&RTT_S0, 
                 "Error: All parameters (IP, gateway, netmask) are required\n");
        return false;
    }
    
    // Parse IP address
    if (!inet_aton(ip_str, &ip_addr)) {
        chprintf((BaseSequentialStream *)&RTT_S0, 
                 "Error: Invalid IP address format: %s\n", ip_str);
        return false;
    }
    
    // Parse gateway address
    if (!inet_aton(gateway_str, &gateway_addr)) {
        chprintf((BaseSequentialStream *)&RTT_S0, 
                 "Error: Invalid gateway address format: %s\n", gateway_str);
        return false;
    }
    
    // Parse netmask
    if (!inet_aton(netmask_str, &netmask_addr)) {
        chprintf((BaseSequentialStream *)&RTT_S0, 
                 "Error: Invalid netmask format: %s\n", netmask_str);
        return false;
    }
    
    // Build reconfiguration structure
    reconf_opts.address = ip_addr.addr;
    reconf_opts.netmask = netmask_addr.addr;
    reconf_opts.gateway = gateway_addr.addr;
    reconf_opts.addrMode = NET_ADDRESS_STATIC;
    
    // Apply the reconfiguration
    lwipReconfigure(&reconf_opts);
    
    // Log the change
    chprintf((BaseSequentialStream *)&RTT_S0, 
             "Network reconfigured to:\n"
             "  IP:      %s\n"
             "  Gateway: %s\n"
             "  Netmask: %s\n",
             ip_str, gateway_str, netmask_str);
    
    return true;
}

/* UDP Destination Configuration Functions
 * 
 * Set the UDP destination IP address for unicast transmission.
 * 
 * Parameters:
 *   dest_ip: Destination IP address string (e.g., "192.168.0.50")
 * 
 * Returns:
 *   true on success, false on failure (invalid IP format)
 * 
 * Example usage:
 *   udp_set_destination("192.168.0.50");   // Send to specific device
 *   udp_set_destination("192.168.0.255");  // Subnet broadcast
 *   udp_set_destination("255.255.255.255"); // Global broadcast
 */
static bool udp_set_destination(const char* dest_ip) {
    if (dest_ip == NULL) {
        return false;
    }
    
    // Validate IP address length
    size_t len = strlen(dest_ip);
    if (len == 0 || len >= sizeof(udp_dest_ip)) {
        return false;
    }
    
    // Validate IP address format
    ip4_addr_t test_addr;
    if (!inet_aton(dest_ip, &test_addr)) {
        chprintf((BaseSequentialStream *)&RTT_S0, 
                 "Error: Invalid IP address format: %s\n", dest_ip);
        return false;
    }
    
    chMtxLock(&udp_dest_mutex);
    strncpy(udp_dest_ip, dest_ip, sizeof(udp_dest_ip) - 1);
    udp_dest_ip[sizeof(udp_dest_ip) - 1] = '\0';
    chMtxUnlock(&udp_dest_mutex);
    
    chprintf((BaseSequentialStream *)&RTT_S0, 
             "UDP destination IP updated to: %s\n", dest_ip);
    
    return true;
}

/* Get UDP destination IP address (thread-safe copy)
 * 
 * Parameters:
 *   dest_ip:  Buffer to store IP address string
 *   buf_size: Size of the buffer
 */
static void udp_get_destination(char* dest_ip, size_t buf_size) {
    if (dest_ip == NULL || buf_size == 0) {
        return;
    }
    
    chMtxLock(&udp_dest_mutex);
    strncpy(dest_ip, udp_dest_ip, buf_size - 1);
    dest_ip[buf_size - 1] = '\0';
    chMtxUnlock(&udp_dest_mutex);
}

/* ===========================================================================
 * THREAD IMPLEMENTATIONS
 * =========================================================================== */

/* ---------------------------------------------------------------------------
 * Thread Monitor Display Thread
 * --------------------------------------------------------------------------- */

static THD_WORKING_AREA(waThreadMonitor, 1024);
static THD_FUNCTION(ThreadMonitor, arg) {
    (void)arg;
    chRegSetThreadName("thread_monitor");
    
    chprintf((BaseSequentialStream *)&RTT_S0, "Thread Monitor started\n");
    
    while (true) {
        chThdSleepMilliseconds(MONITOR_DISPLAY_INTERVAL_MS);
        
        chprintf((BaseSequentialStream *)&RTT_S0, "\n=== THREAD LOOP TIME & FREQUENCY STATISTICS ===\n");
        chprintf((BaseSequentialStream *)&RTT_S0, "Thread Name        | Loops | Avg(ms) | Min(ms) | Max(ms) | Last(ms) | Freq(Hz)\n");
        chprintf((BaseSequentialStream *)&RTT_S0, "-------------------|-------|---------|---------|---------|---------|--------\n");
        
        for (uint8_t i = 0; i < monitor_count; i++) {
            if (thread_monitors[i].is_active) {
                thread_monitor_t* monitor = &thread_monitors[i];
                systime_t avg_loop_time = 0;
                float frequency_hz = 0.0f;
                
                if (monitor->loop_count > 0) {
                    avg_loop_time = monitor->total_loop_time / monitor->loop_count;
                }
                
                // Calculate frequency based on wake-up count and time elapsed
                if (monitor->wake_count > 1 && monitor->first_loop_time > 0) {
                    systime_t total_time = monitor->last_wake_time - monitor->first_loop_time;
                    if (total_time > 0) {
                        // Convert to frequency: wake_count / (total_time_in_seconds)
                        frequency_hz = (float)monitor->wake_count / ((float)TIME_I2MS(total_time) / 1000.0f);
                    }
                }
                
                chprintf((BaseSequentialStream *)&RTT_S0, 
                    "%-18s | %5lu | %7lu | %7lu | %7lu | %7lu | %6.1f\n",
                    monitor->name,
                    (unsigned long)monitor->loop_count,
                    (unsigned long)TIME_I2MS(avg_loop_time),
                    (unsigned long)TIME_I2MS(monitor->min_loop_time),
                    (unsigned long)TIME_I2MS(monitor->max_loop_time),
                    (unsigned long)TIME_I2MS(monitor->loop_duration),
                    frequency_hz
                );
            }
        }
        chprintf((BaseSequentialStream *)&RTT_S0, "=====================================\n\n");
    }
}

/* ---------------------------------------------------------------------------
 * LED Blinker Thread
 * --------------------------------------------------------------------------- */

static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  
  while (true) {
    update_thread_loop_start("blinker");
    
    palClearLine(LINE_LED_RED_E12);
    chThdSleepMilliseconds(1000);
    palSetLine(LINE_LED_RED_E12);
    chThdSleepMilliseconds(1000);
    
    update_thread_loop_end("blinker");
  }
}

/* ---------------------------------------------------------------------------
 * USB HID Threads
 * --------------------------------------------------------------------------- */

#if HAL_USBH_USE_HID

/* HID Report Callback (ISR context)
 * NOTE: This is an ISR callback, must use ISR class calls and lock/unlock
 * PS5 callback frequency should be around 250 Hz
 * Reference: https://github.com/Ryochan7/DS4Windows/issues/1608 */
static void _hid_report_callback(USBHHIDDriver *hidp, uint16_t len) {
    chSysLockFromISR();
    uint8_t *report = (uint8_t *)hidp->config->report_buffer;
    
    /* Try DualSense direct decode → publish latest-only */
    uint8_t next = ds5PublishedIndex ^ 1;  /* back buffer */
    if (ds5_from_hid_report(report, len, &ds5Buf[next])) {
        ds5PublishedIndex = next;          /* publish */
        ds5Generation++;
        chEvtBroadcastI(&esDS5Ready);      /* notify all consumers */
    }
    chSysUnlockFromISR();
}

/* HID Management Thread */
static THD_WORKING_AREA(waTestHID, 1024);
static void ThreadTestHID(void *p) {
  (void)p;
  uint8_t i;
  static uint8_t kbd_led_states[HAL_USBHHID_MAX_INSTANCES];

  chRegSetThreadName("HID");

  for (i = 0; i < HAL_USBHHID_MAX_INSTANCES; i++) {
      hidcfg[i].cb_report = _hid_report_callback;
      hidcfg[i].protocol = USBHHID_PROTOCOL_REPORT;
      hidcfg[i].report_buffer = report[i];
      hidcfg[i].report_len = 64;
  }

    chprintf((BaseSequentialStream *)&RTT_S0, "HID Thread started\n");

  for (;;) {
      update_thread_loop_start("HID");
      
      for (i = 0; i < HAL_USBHHID_MAX_INSTANCES; i++) {
          USBHHIDDriver *const hidp = &USBHHIDD[i];
          usbhhid_state_t state = usbhhidGetState(hidp);
          
          if (state == USBHHID_STATE_ACTIVE) {
              chprintf((BaseSequentialStream *)&RTT_S0, "HID: Connected, HID%d\n", i);
              usbhhidStart(hidp, &hidcfg[i]);
              if (usbhhidGetType(hidp) != USBHHID_DEVTYPE_GENERIC) {
                  usbhhidSetIdle(hidp, 0, 0);
              }
              kbd_led_states[i] = 1;
          } else if (state == USBHHID_STATE_READY) {
                // Device is ready but no action needed
          }
      }
      chThdSleepMilliseconds(500);
      
      update_thread_loop_end("HID");
  }
}

/* USB Host Main Loop Thread
 * Dedicated thread for USB host operations with high-frequency polling
 * NOTE: There are known USB FS timeout issues for -Os build
 * Reference: https://community.st.com/t5/stm32-mcus/faq-troubleshooting-a-usb-core-soft-reset-stuck-on-an-stm32/ta-p/803224 */
static THD_WORKING_AREA(waUsbHost, 1024);
static THD_FUNCTION(UsbHostThread, arg) {
  (void)arg;
  chRegSetThreadName("usb_host");
  chEvtObjectInit(&esDS5Ready);
  usbhStart(&USBHD2);
  chprintf((BaseSequentialStream *)&RTT_S0, "USB Host Thread started\n");
  
  while (true) {
    update_thread_loop_start("usb_host");
    
    #if STM32_USBH_USE_OTG2
      usbhMainLoop(&USBHD2);
    #endif
    #if STM32_USBH_USE_OTG1
      usbhMainLoop(&USBHD1);
    #endif
        chThdSleepMilliseconds(100);  // 100ms polling for USB events
    
    update_thread_loop_end("usb_host");
  }
}

#endif /* HAL_USBH_USE_HID */

/* ---------------------------------------------------------------------------
 * Network Thread
 * --------------------------------------------------------------------------- */

/* UDP Server Thread - sends PS5 controller data over Ethernet (unicast) */
static THD_WORKING_AREA(waUdpServer, 1024);
static THD_FUNCTION(UdpServerThread, arg) {
  (void)arg;
  chRegSetThreadName("udp_server");
  
    // Network configuration
  uint8_t mac_address[6] = {0x02, 0x12, 0x13, 0x10, 0x15, 0x05};
  
  ip4_addr_t ip_addr, gateway_addr, netmask_addr;
    IP4_ADDR(&ip_addr, 192, 168, 0, 100);        // IP: 192.168.0.100
    IP4_ADDR(&gateway_addr, 192, 168, 0, 1);     // Gateway: 192.168.0.1
    IP4_ADDR(&netmask_addr, 255, 255, 255, 0);   // Netmask: 255.255.255.0

  // Set up the lwIP thread options
  lwipthread_opts_t lwipthread_opts = {
        .macaddress = mac_address,
        .address = ip_addr.addr,
        .netmask = netmask_addr.addr,
        .gateway = gateway_addr.addr,
        .addrMode = NET_ADDRESS_STATIC,
        .ourHostName = "ds-eth-comm",
        .link_up_cb = myLinkUpCallback,
        .link_down_cb = myLinkDownCallback
  };

  // Initialize lwIP network stack
  lwipInit(&lwipthread_opts);
  chprintf((BaseSequentialStream *)&RTT_S0, "Network initialized in UDP thread\n");
  
  // Initialize UDP destination mutex
  chMtxObjectInit(&udp_dest_mutex);
  
  event_listener_t el;
  chEvtRegisterMask(&esDS5Ready, &el, EVT_DS5_READY);
  
  int sock;
  struct sockaddr_in server_addr;
  struct sockaddr_in dest_addr;
  
  // Create UDP socket
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    chprintf((BaseSequentialStream *)&RTT_S0, "Failed to create UDP socket\n");
    return;
  }
  
  // Configure server address
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(UDP_SERVER_PORT);
  
  // Bind socket to address
  if (bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    chprintf((BaseSequentialStream *)&RTT_S0, "Failed to bind UDP socket to port %d\n", UDP_SERVER_PORT);
    close(sock);
    return;
  }
  
  chprintf((BaseSequentialStream *)&RTT_S0, "UDP Server started on port %d\n", UDP_SERVER_PORT);
  
  // Get initial destination IP
  char dest_ip_str[16];
  udp_get_destination(dest_ip_str, sizeof(dest_ip_str));
  chprintf((BaseSequentialStream *)&RTT_S0, "UDP destination: %s:%d\n", dest_ip_str, JOY_STREAM_PORT);
  
  while (true) {
    update_thread_loop_start("udp_server");
    
    /* Wait for new DS5 data when idle; busy processing will naturally coalesce */
    chEvtWaitOne(EVT_DS5_READY);
    uint32_t g = ds5Generation;
    uint8_t idx = ds5PublishedIndex;
    /* Snapshot to stack to avoid partial reads if another publish happens */
    PS5Data snap;
    convert_to_trimmed(&ds5Buf[idx], &snap);
    (void)g; /* keep for future per-consumer versioning if needed */

#if JOY_STREAM_ENABLE
    // Get current destination IP (thread-safe)
    char current_dest_ip[16];
    udp_get_destination(current_dest_ip, sizeof(current_dest_ip));
    
    // Configure destination address
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    if (!inet_aton(current_dest_ip, &dest_addr.sin_addr)) {
        // If invalid IP, skip this send
    update_thread_loop_end("udp_server");
        continue;
    }
    dest_addr.sin_port = htons(JOY_STREAM_PORT);
    
    // Send trimmed PS5Data to configured destination (unicast)
    sendto(sock, (const void*)&snap, sizeof(PS5Data), 0, 
           (struct sockaddr*)&dest_addr, sizeof(dest_addr));
#endif
    
    update_thread_loop_end("udp_server");
  }
}

/* ---------------------------------------------------------------------------
 * nRF24L01 Threads
 * --------------------------------------------------------------------------- */

/* nRF24L01 Interrupt Handling Thread */
static THD_WORKING_AREA(waNRF24Interrupt, 2048);
static THD_FUNCTION(NRF24InterruptThread, arg) {
    (void)arg;
    
    chRegSetThreadName("nrf24_irq");
    
    // Register for interrupt events
    chEvtRegisterMask(&esNRF24Interrupt, &nrf24_interrupt_listener, EVT_NRF24_INTERRUPT);
    
    while (true) {
        update_thread_loop_start("nrf24_irq");
        
        // Wait for interrupt event
        eventmask_t evt = chEvtWaitOne(EVT_NRF24_INTERRUPT);
        
        if (evt & EVT_NRF24_INTERRUPT) {
            // Handle the NRF24L01 interrupt
            uint8_t result = nrf24l01_interrupt_irq_handler();
            
            if (result != 0) {
                nrf24l01_interface_debug_print("nrf24l01: interrupt handler failed with code %d\n", result);
            }
        }
        
        update_thread_loop_end("nrf24_irq");
    }
}

/* nRF24 TX Broadcaster Thread - Sends latest DS5 snapshot over nRF24 in 32-byte chunks */
static THD_WORKING_AREA(waNRF24Tx, 2048);
static THD_FUNCTION(NRF24TxThread, arg) {
  (void)arg;
  chRegSetThreadName("nrf24_tx");
  event_listener_t el;
    
  /* Initialize NRF24L01 interrupt event source */
  chEvtObjectInit(&esNRF24Interrupt);

  nrf24l01_basic_init(NRF24L01_TYPE_TX, interrupt_callback);
  
  palEnableLineEvent(LINE_SPI2_NRF24_IRQ, PAL_EVENT_MODE_FALLING_EDGE);
  palSetLineCallback(LINE_SPI2_NRF24_IRQ, nrf24l01_interface_gpio_interrupt_callback, NULL);
  
  chEvtRegisterMask(&esDS5Ready, &el, EVT_DS5_READY);

    /* nRF24L01 TX destination address: "2Node" (check endianness) */
    uint8_t nrf_tx_addr[5] = {0x65, 0x64, 0x6F, 0x4E, 0x32};
    nrf24l01_basic_set_tx_address(nrf_tx_addr);

  while (true) {
    update_thread_loop_start("nrf24_tx");
    
    chEvtWaitOne(EVT_DS5_READY);

    uint8_t idx = ds5PublishedIndex;
    
    // Convert full PS5DataFull to trimmed PS5Data for transmission
    PS5Data trimmed_data;
    convert_to_trimmed(&ds5Buf[idx], &trimmed_data);

        // Uncomment to debug: print_ps5data_hex(&trimmed_data, "PS5Data Structure");

        /* Blocking call if use_ack is TRUE - will wait for the ack from the receiver */
    if (nrf24l01_basic_send(nrf_tx_addr, (uint8_t *)&trimmed_data, sizeof(PS5Data), NRF24L01_BOOL_FALSE) != 0) {
            // Handle transmission error if needed
    }
    
    update_thread_loop_end("nrf24_tx");
  }
}

/* ===========================================================================
 * SHELL COMMANDS
 * =========================================================================== */

/* Shell command: Set nRF24L01 channel frequency
 * Usage: nrf_channel <freq>
 * Example: nrf_channel 94
 *          nrf_channel 100
 */
static void cmd_nrf_channel(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 1) {
        chprintf(chp, "Usage: nrf_channel <freq>\r\n");
        chprintf(chp, "  freq: Channel frequency (0-127)\r\n");
        chprintf(chp, "  Actual frequency = 2400 MHz + freq MHz\r\n");
        chprintf(chp, "Example: nrf_channel 94  (sets to 2494 MHz)\r\n");
        return;
    }
    
    int freq = atoi(argv[0]);
    
    if (freq < 0 || freq > 127) {
        chprintf(chp, "Error: Frequency must be between 0 and 127\r\n");
        return;
    }
    
    uint8_t result = nrf24l01_basic_set_frequency((uint8_t)freq);
    
    if (result == 0) {
        chprintf(chp, "nRF24L01 channel set to %d (frequency: %d MHz)\r\n", 
                 freq, 2400 + freq);
    } else {
        chprintf(chp, "Error: Failed to set nRF24L01 channel\r\n");
    }
}

/* Shell command: Get current nRF24L01 channel frequency
 * Usage: nrf_channel_get
 */
static void cmd_nrf_channel_get(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    
    if (argc != 0) {
        chprintf(chp, "Usage: nrf_channel_get\r\n");
        chprintf(chp, "  Displays the current nRF24L01 channel frequency\r\n");
        return;
    }
    
    uint8_t freq;
    uint8_t result = nrf24l01_basic_get_frequency(&freq);
    
    if (result == 0) {
        chprintf(chp, "Current nRF24L01 channel: %d (frequency: %d MHz)\r\n", 
                 freq, 2400 + freq);
    } else {
        chprintf(chp, "Error: Failed to get nRF24L01 channel\r\n");
    }
}

/* Shell command: Set nRF24L01 TX address
 * Usage: nrf_addr <byte0> <byte1> <byte2> <byte3> <byte4>
 * Example: nrf_addr 0x65 0x64 0x6F 0x4E 0x32
 *          nrf_addr 0xE7 0xE7 0xE7 0xE7 0xE7
 */
static void cmd_nrf_addr(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 5) {
        chprintf(chp, "Usage: nrf_addr <byte0> <byte1> <byte2> <byte3> <byte4>\r\n");
        chprintf(chp, "  Sets the nRF24L01 TX address (5 bytes in hex)\r\n");
        chprintf(chp, "Example: nrf_addr 0x65 0x64 0x6F 0x4E 0x32\r\n");
        chprintf(chp, "         nrf_addr 0xE7 0xE7 0xE7 0xE7 0xE7\r\n");
        return;
    }
    
    uint8_t addr[5];
    
    // Parse 5 hex bytes
    for (int i = 0; i < 5; i++) {
        char *endptr;
        long val = strtol(argv[i], &endptr, 0);  // Auto-detect base (0x for hex)
        
        if (*endptr != '\0' || val < 0 || val > 255) {
            chprintf(chp, "Error: Invalid byte %d: %s (must be 0x00-0xFF)\r\n", i, argv[i]);
            return;
        }
        addr[i] = (uint8_t)val;
    }
    
    uint8_t result = nrf24l01_basic_set_tx_address(addr);
    
    if (result == 0) {
        chprintf(chp, "nRF24L01 TX address set to: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
                 addr[0], addr[1], addr[2], addr[3], addr[4]);
    } else {
        chprintf(chp, "Error: Failed to set nRF24L01 TX address\r\n");
    }
}

/* Shell command: Get current nRF24L01 TX address
 * Usage: nrf_addr_get
 */
static void cmd_nrf_addr_get(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    
    if (argc != 0) {
        chprintf(chp, "Usage: nrf_addr_get\r\n");
        chprintf(chp, "  Displays the current nRF24L01 TX address\r\n");
        return;
    }
    
    uint8_t addr[5];
    uint8_t is_set;
    uint8_t result = nrf24l01_basic_get_tx_address(addr, &is_set);
    
    if (result == 0) {
        if (is_set) {
            chprintf(chp, "Current nRF24L01 TX address: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
                     addr[0], addr[1], addr[2], addr[3], addr[4]);
            
            // Try to display as ASCII if printable
            bool all_printable = true;
            for (int i = 0; i < 5; i++) {
                if (addr[i] < 32 || addr[i] > 126) {
                    all_printable = false;
                    break;
                }
            }
            if (all_printable) {
                chprintf(chp, "                     (ASCII: '%c%c%c%c%c')\r\n",
                         addr[0], addr[1], addr[2], addr[3], addr[4]);
            }
        } else {
            chprintf(chp, "nRF24L01 TX address has not been set yet\r\n");
        }
    } else {
        chprintf(chp, "Error: Failed to get nRF24L01 TX address\r\n");
    }
}

/* Shell command: Configure network IP settings
 * Usage: net_config <ip> <gateway> <netmask>
 * Example: net_config 192.168.0.150 192.168.0.1 255.255.255.0
 *          net_config 10.0.0.50 10.0.0.1 255.255.255.0
 */
static void cmd_net_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 3) {
        chprintf(chp, "Usage: net_config <ip> <gateway> <netmask>\r\n");
        chprintf(chp, "Example: net_config 192.168.0.150 192.168.0.1 255.255.255.0\r\n");
        chprintf(chp, "         net_config 10.0.0.50 10.0.0.1 255.255.255.0\r\n");
        return;
    }
    
    const char* ip_str = argv[0];
    const char* gateway_str = argv[1];
    const char* netmask_str = argv[2];
    
    chprintf(chp, "Configuring network to:\r\n");
    chprintf(chp, "  IP:      %s\r\n", ip_str);
    chprintf(chp, "  Gateway: %s\r\n", gateway_str);
    chprintf(chp, "  Netmask: %s\r\n", netmask_str);
    
    bool result = network_reconfigure(ip_str, gateway_str, netmask_str);
    
    if (result) {
        chprintf(chp, "Network configuration applied successfully!\r\n");
        chprintf(chp, "Note: Existing connections may need to be reestablished.\r\n");
    } else {
        chprintf(chp, "Error: Failed to configure network (check IP format)\r\n");
    }
}

/* Shell command: Get current network IP settings
 * Usage: net_info
 */
static void cmd_net_info(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    
    if (argc != 0) {
        chprintf(chp, "Usage: net_info\r\n");
        chprintf(chp, "  Displays the current network configuration\r\n");
        return;
    }
    
    // Get the default netif (first network interface)
    struct netif *netif = netif_default;
    
    if (netif == NULL) {
        chprintf(chp, "Error: Network interface not initialized\r\n");
        return;
    }
    
    chprintf(chp, "Current Network Configuration:\r\n");
    chprintf(chp, "  Interface:  %c%c%d\r\n", 
             netif->name[0], netif->name[1], netif->num);
    chprintf(chp, "  IP Address: %d.%d.%d.%d\r\n",
             ip4_addr1(&netif->ip_addr), ip4_addr2(&netif->ip_addr),
             ip4_addr3(&netif->ip_addr), ip4_addr4(&netif->ip_addr));
    chprintf(chp, "  Gateway:    %d.%d.%d.%d\r\n",
             ip4_addr1(&netif->gw), ip4_addr2(&netif->gw),
             ip4_addr3(&netif->gw), ip4_addr4(&netif->gw));
    chprintf(chp, "  Netmask:    %d.%d.%d.%d\r\n",
             ip4_addr1(&netif->netmask), ip4_addr2(&netif->netmask),
             ip4_addr3(&netif->netmask), ip4_addr4(&netif->netmask));
    chprintf(chp, "  MAC:        %02X:%02X:%02X:%02X:%02X:%02X\r\n",
             netif->hwaddr[0], netif->hwaddr[1], netif->hwaddr[2],
             netif->hwaddr[3], netif->hwaddr[4], netif->hwaddr[5]);
    chprintf(chp, "  Link:       %s\r\n", 
             netif_is_link_up(netif) ? "UP" : "DOWN");
    chprintf(chp, "  Status:     %s\r\n", 
             netif_is_up(netif) ? "UP" : "DOWN");
}

/* Shell command: Set UDP destination IP
 * Usage: udp_dest <ip>
 * Example: udp_dest 192.168.0.50
 *          udp_dest 192.168.0.255
 */
static void cmd_udp_dest(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 1) {
        chprintf(chp, "Usage: udp_dest <ip>\r\n");
        chprintf(chp, "  Sets the UDP destination IP address\r\n");
        chprintf(chp, "Example: udp_dest 192.168.0.50    (unicast to specific device)\r\n");
        chprintf(chp, "         udp_dest 192.168.0.255   (subnet broadcast)\r\n");
        chprintf(chp, "         udp_dest 255.255.255.255 (global broadcast)\r\n");
        return;
    }
    
    const char* dest_ip = argv[0];
    
    bool result = udp_set_destination(dest_ip);
    
    if (result) {
        chprintf(chp, "UDP destination set to: %s\r\n", dest_ip);
        chprintf(chp, "PS5 controller data will now be sent to %s:%d\r\n", 
                 dest_ip, JOY_STREAM_PORT);
    } else {
        chprintf(chp, "Error: Failed to set UDP destination (invalid IP format)\r\n");
    }
}

/* Shell command: Get UDP destination IP
 * Usage: udp_dest_get
 */
static void cmd_udp_dest_get(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    
    if (argc != 0) {
        chprintf(chp, "Usage: udp_dest_get\r\n");
        chprintf(chp, "  Displays the current UDP destination IP address\r\n");
        return;
    }
    
    char dest_ip[16];
    udp_get_destination(dest_ip, sizeof(dest_ip));
    
    chprintf(chp, "Current UDP destination: %s:%d\r\n", dest_ip, JOY_STREAM_PORT);
}

/* ===========================================================================
 * SHELL CONFIGURATION
 * =========================================================================== */

static const ShellCommand commands[] = {
    {"nrf_channel", cmd_nrf_channel},
    {"nrf_channel_get", cmd_nrf_channel_get},
    {"nrf_addr", cmd_nrf_addr},
    {"nrf_addr_get", cmd_nrf_addr_get},
    {"net_config", cmd_net_config},
    {"net_info", cmd_net_info},
    {"udp_dest", cmd_udp_dest},
    {"udp_dest_get", cmd_udp_dest_get},
    {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
    (BaseSequentialStream *)&RTT_S0,
    commands,
    shell_history,
    sizeof(shell_history),
    shell_completions,
};

/* ===========================================================================
 * MAIN FUNCTION
 * =========================================================================== */

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  RTTchannelObjectInit(&RTT_S0);

    /* FlashDB initialization (disabled)
    uint8_t res = flashdb_init();
    chprintf((BaseSequentialStream *)&RTT_S0, "flashdb_init result: %d\n", res);
    char boot_count = 0;
    flashdb_get_kv_value("boot_count", &boot_count, sizeof(boot_count));
    boot_count++;
    flashdb_set_kv_value("boot_count", &boot_count, sizeof(boot_count));
    chprintf((BaseSequentialStream *)&RTT_S0, "Boot count: %d\n", boot_count);
    */

    /* Create all system threads */
    thread_t* blinker_tp = chThdCreateStatic(waThread1, sizeof(waThread1), 
                                              NORMALPRIO+1, Thread1, NULL);
    thread_t* hid_tp = chThdCreateStatic(waTestHID, sizeof(waTestHID), 
                                         NORMALPRIO, ThreadTestHID, NULL);
    thread_t* usb_host_tp = chThdCreateStatic(waUsbHost, sizeof(waUsbHost), 
                                               NORMALPRIO+2, UsbHostThread, NULL);
    thread_t* udp_server_tp = chThdCreateStatic(waUdpServer, sizeof(waUdpServer), 
                                                 NORMALPRIO, UdpServerThread, NULL);
    thread_t* nrf24_tx_tp = chThdCreateStatic(waNRF24Tx, sizeof(waNRF24Tx), 
                                               NORMALPRIO, NRF24TxThread, NULL);
    thread_t* nrf24_irq_tp = chThdCreateStatic(waNRF24Interrupt, sizeof(waNRF24Interrupt), 
                                                NORMALPRIO+3, NRF24InterruptThread, NULL);
    
    /* Register threads for monitoring */
  register_thread_monitor("blinker", blinker_tp);
  register_thread_monitor("udp_server", udp_server_tp);
  register_thread_monitor("nrf24_tx", nrf24_tx_tp);
  register_thread_monitor("nrf24_irq", nrf24_irq_tp);
  register_thread_monitor("HID", hid_tp);
  register_thread_monitor("usb_host", usb_host_tp);
  
    /* Create and start the thread monitor */
    // chThdCreateStatic(waThreadMonitor, sizeof(waThreadMonitor), 
    //                   NORMALPRIO-1, ThreadMonitor, NULL);
  
    /* Main thread becomes shell manager */
  while (1) {
      thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
      "shell", NORMALPRIO + 1,
      shellThread, (void *)&shell_cfg1);
        chThdWait(shelltp);               /* Wait for termination */
    chThdSleepMilliseconds(500);
  }
}
