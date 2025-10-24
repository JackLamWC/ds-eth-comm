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

/* ChibiOS Core */
#include "ch.h"
#include "hal.h"
#include "chprintf.h"


/* FlashDB */
#include "fdb_port.h"

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

/* Hardware Drivers */
#include "w25qxx_interface.h"
#include "nrf24l01.h"
#include "nrf24l01_interface.h"
#include "nrf24l01_basic.h"

/* Shell */
#include "shell.h"

/* ============================================================================
 * PS5 CONTROLLER CONSTANTS
 * ============================================================================ */

// Button constants for PS5 controller
#define UP 0x00
#define RIGHT 0x01
#define DOWN 0x02
#define LEFT 0x03

// PS5 Button mapping array
const uint8_t PS5_BUTTONS[] = {
        UP, // UP
        RIGHT, // RIGHT
        DOWN, // DOWN
        LEFT, // LEFT

        0x0C, // CREATE
        0x0D, // OPTIONS
        0x0E, // L3
        0x0F, // R3

        0x0A, // L2
        0x0B, // R2
        0x08, // L1
        0x09, // R1

        0x07, // TRIANGLE
        0x06, // CIRCLE
        0x05, // CROSS
        0x04, // SQUARE

        0x10, // PS
        0x11, // TOUCHPAD
        0x12, // MICROPHONE
};

/* ============================================================================
 * GLOBAL VARIABLES AND CONSTANTS
 * ============================================================================ */


/* Network Configuration */
#define UDP_SERVER_PORT    12345
#define UDP_BUFFER_SIZE    1024
#define JOY_STREAM_ENABLE  1
#define JOY_STREAM_PORT    12346

/* Shell Configuration */
char shell_history[SHELL_MAX_HIST_BUFF];
char *shell_completions[SHELL_MAX_COMPLETIONS];
#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(128)

/* ============================================================================
 * DUALSENSE CONTROLLER STRUCTURES AND ENUMS
 * ============================================================================ */

#if HAL_USBH_USE_HID

/* DualSense (DS5) input report core layout (USB payload).
 * This maps sticks, triggers, buttons, IMU, and touchpad in a compact form
 * and can be used to parse DS5 reports once positioned at the payload start. */
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

union DS5Buttons {
  struct {
    uint8_t dpad : 4;       /* ds5_dpad_t */
    uint8_t square : 1;
    uint8_t cross : 1;
    uint8_t circle : 1;
    uint8_t triangle : 1;

    uint8_t l1 : 1;
    uint8_t r1 : 1;
    uint8_t l2 : 1;         /* digital */
    uint8_t r2 : 1;         /* digital */
    uint8_t create : 1;     /* Share/Create */
    uint8_t options : 1;    /* Options */
    uint8_t l3 : 1;
    uint8_t r3 : 1;

    uint8_t ps : 1;         /* PS button */
    uint8_t touchpad : 1;   /* Touchpad press */
    uint8_t mic : 1;        /* Mic mute */
    uint8_t reserved : 5;
  } __attribute__((packed));
  uint32_t val : 24;
} __attribute__((packed));

struct DS5TouchpadXY {
  struct {
    uint8_t counter : 7;    /* increments while finger active */
    uint8_t touching : 1;   /* 0 = touching, 1 = not touching */
    uint16_t x : 12;        /* 0..1919 */
    uint16_t y : 12;        /* 0..1079 */
  } __attribute__((packed)) finger[2];
} __attribute__((packed));

// PS5 Button definitions
union PS5Buttons {
        struct {
                uint8_t dpad : 4;
                uint8_t square : 1;
                uint8_t cross : 1;
                uint8_t circle : 1;
                uint8_t triangle : 1;

                uint8_t l1 : 1;
                uint8_t r1 : 1;
                uint8_t l2 : 1;
                uint8_t r2 : 1;
                uint8_t create : 1;
                uint8_t menu : 1;
                uint8_t l3 : 1;
                uint8_t r3 : 1;

                uint8_t ps : 1;
                uint8_t touchpad : 1;
                uint8_t mic : 1;
                uint8_t dummy : 5;
        } __attribute__((packed));
        uint32_t val : 24;
} __attribute__((packed));

// PS5 Touchpad definitions
struct ps5TouchpadXY {
        struct {
                uint8_t counter : 7; // Increments every time a finger is touching the touchpad
                uint8_t touching : 1; // The top bit is cleared if the finger is touching the touchpad
                uint16_t x : 12;
                uint16_t y : 12;
        } __attribute__((packed)) finger[2]; // 0 = first finger, 1 = second finger
} __attribute__((packed));

// PS5 Status definitions
union PS5Status {
        struct {
                // first byte
                uint8_t headphone : 1;
                uint8_t dummy : 2; // Seems to change when a jack is plugged in. First bit stays on when a mic is plugged in.
                uint8_t usb : 1; // charging
                uint8_t dummy2: 4;

                // second byte
                uint8_t mic : 1;
                uint8_t dummy3 : 3;
        } __attribute__((packed));
        uint16_t val;
} __attribute__((packed));

typedef union PS5Buttons PS5Buttons;
typedef struct ps5TouchpadXY ps5TouchpadXY;
typedef union PS5Status PS5Status;

// Full PS5Data structure (40 bytes) - for internal use
struct PS5DataFull {
        /* Button and joystick values */
        uint8_t hatValue[4]; // 0-3 bytes
        uint8_t trigger[2]; // 4-5

        uint8_t sequence_number; // 6

        PS5Buttons btn; // 7-9

        uint8_t reserved[5]; // 0xA-0xD

        /* Gyro and accelerometer values */
        int16_t gyroX, gyroZ, gyroY; // 0x0F - 0x14
        int16_t accX, accZ, accY; // 0x15-0x1A
        int32_t sensor_timestamp; 

        uint8_t reserved2;

        // 0x20 - 0x23 touchpad point 1
        // 0x24 - 0x27 touchpad point 2
        ps5TouchpadXY xy;

#if 0 // The status byte depends on if it's sent via USB or Bluetooth, so is not parsed for now
        uint8_t reserved3; // 0x28

        uint8_t rightTriggerFeedback; // 0x29
        uint8_t leftTriggerFeedback; // 0x2A
        uint8_t reserved4[10]; // 0x2B - 0x34

        // status bytes 0x35-0x36
        PS5Status status;
#endif
} __attribute__((packed));

// Trimmed PS5Data structure (31 bytes) - for nRF24L01 transmission
// Size: hatValue[4] + trigger[2] + sequence_number[1] + btn[3] + reserved[5] + 
//       gyroX,gyroZ,gyroY[6] + accX,accZ,accY[6] + sensor_timestamp[4] = 31 bytes
struct PS5Data {
        /* Button and joystick values */
        uint8_t hatValue[4]; // 0-3 bytes
        uint8_t trigger[2]; // 4-5

        uint8_t sequence_number; // 6

        PS5Buttons btn; // 7-9

        uint8_t reserved[5]; // 0xA-0xD

        /* Gyro and accelerometer values */
        int16_t gyroX, gyroZ, gyroY; // 0x0F - 0x14
        int16_t accX, accZ, accY; // 0x15-0x1A
        int32_t sensor_timestamp; // 0x1B-0x1E
} __attribute__((packed));

typedef struct PS5Data PS5Data;
typedef struct PS5DataFull PS5DataFull;

/* Latest-only publication via double buffer + event broadcast */
#define EVT_DS5_READY           EVENT_MASK(0)
#define EVT_NRF24_INTERRUPT     EVENT_MASK(1)

static PS5DataFull ds5Buf[2];
static volatile uint8_t ds5PublishedIndex = 0;
static volatile uint32_t ds5Generation = 0;
static event_source_t esDS5Ready;

/* Parse raw HID report buffer into PS5DataFull.
 * Supports USB (report ID 0x01) and Bluetooth (report ID 0x31).
 * Returns true on success. */
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

/* Print hex dump of PS5Data structure */
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
#endif

/* ============================================================================
 * INTERFACE CALLBACK FUNCTIONS
 * ============================================================================ */

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

/* ============================================================================
 * nRF24L01 INTERRUPT CALLBACK
 * ============================================================================ */

static void interrupt_callback(uint8_t type, uint8_t num, uint8_t *buf, uint8_t len)
{
    switch (type)
    {
        case NRF24L01_INTERRUPT_RX_DR :
        {
            uint8_t i;
            
            nrf24l01_interface_debug_print("nrf24l01: irq receive with pipe %d with %d.\n", num, len);
            for (i = 0; i < len; i++)
            {
                nrf24l01_interface_debug_print("0x%02X ", buf[i]);
            }
            nrf24l01_interface_debug_print(".\n");
            
            break;
        }
        case NRF24L01_INTERRUPT_TX_DS :
        {
            nrf24l01_interface_debug_print("nrf24l01: irq send ok.\n");
            
            break;
        }
        case NRF24L01_INTERRUPT_MAX_RT :
        {
            nrf24l01_interface_debug_print("nrf24l01: irq reach max retry times.\n");
            
            break;
        }
        case NRF24L01_INTERRUPT_TX_FULL :
        {
            break;
        }
        default :
        {
            break;
        }
    }
}

/* ============================================================================
 * nRF24L01 INTERRUPT HANDLING THREAD
 * ============================================================================ */

// Event source for NRF24L01 interrupt
static event_source_t esNRF24Interrupt;
static event_listener_t nrf24_interrupt_listener;

// Working area for NRF24L01 interrupt thread
static THD_WORKING_AREA(waNRF24Interrupt, 128);

// NRF24L01 interrupt handling thread
static THD_FUNCTION(NRF24InterruptThread, arg) {
  (void)arg;
  
  chRegSetThreadName("nrf24_irq");
  
  // Register for interrupt events
  chEvtRegisterMask(&esNRF24Interrupt, &nrf24_interrupt_listener, EVT_NRF24_INTERRUPT);
  
  while (true) {
    // Wait for interrupt event
    eventmask_t evt = chEvtWaitOne(EVT_NRF24_INTERRUPT);
    
    if (evt & EVT_NRF24_INTERRUPT) {
      // // Handle the NRF24L01 interrupt
      uint8_t result = nrf24l01_interrupt_irq_handler();
      
      if (result != 0) {
        nrf24l01_interface_debug_print("nrf24l01: interrupt handler failed with code %d\n", result);
      }
      else {
        chprintf((BaseSequentialStream *)&RTT_S0, "nrf24l01: interrupt handler success\n");
      }
    }
  }
}

void nrf24l01_interface_gpio_interrupt_callback(void *args) {
  (void)args;
  chSysLockFromISR();  //! chEvtBroadcastI need to be protected by a critical section
  chEvtBroadcastI(&esNRF24Interrupt); //! rememeber use I class call in interrupt handler
  chSysUnlockFromISR();
}


/* ============================================================================
 * THREAD FUNCTIONS
 * ============================================================================ */

/* LED Blinker Thread */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearLine(LINE_LED_RED_E12);
    chThdSleepMilliseconds(1000);
    palSetLine(LINE_LED_RED_E12);
    chThdSleepMilliseconds(1000);
  }
}

/* ============================================================================
 * CONFIGURATION STRUCTURES
 * ============================================================================ */

/* Shell Configuration */
static const ShellCommand commands[] = {
    {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
    (BaseSequentialStream *)&RTT_S0,
    commands,
    shell_history,
    sizeof(shell_history),
    shell_completions,
};



/* ============================================================================
 * USB HID THREAD FUNCTIONS
 * ============================================================================ */

#if HAL_USBH_USE_HID
#include "usbh/dev/hid.h"


static THD_WORKING_AREA(waUsbHost, 256);

static USBH_DEFINE_BUFFER(uint8_t report[HAL_USBHHID_MAX_INSTANCES][64]);
static USBHHIDConfig hidcfg[HAL_USBHHID_MAX_INSTANCES];

static void _hid_report_callback(USBHHIDDriver *hidp, uint16_t len) {
    uint8_t *report = (uint8_t *)hidp->config->report_buffer;

    /* Try DualSense direct decode → publish latest-only */
    uint8_t next = ds5PublishedIndex ^ 1;            /* back buffer */
    if (ds5_from_hid_report(report, len, &ds5Buf[next])) {
        ds5PublishedIndex = next;                    /* publish */
        ds5Generation++;
        chEvtBroadcast(&esDS5Ready);                 /* notify all consumers */
    }
}

static THD_WORKING_AREA(waTestHID, 256);
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

  chprintf((BaseSequentialStream *)&RTT_S0, "HID Thread started\n");  // Add this

  for (;;) {
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
              // ... rest of your code
          }
      }
      chThdSleepMilliseconds(500);
  }
}

/*
 * USB Host Thread
 * Dedicated thread for USB host operations with high-frequency polling
 */
static THD_FUNCTION(UsbHostThread, arg) {
  (void)arg;
  chRegSetThreadName("usb_host");
  chprintf((BaseSequentialStream *)&RTT_S0, "USB Host Thread started\n");
  
  while (true) {
    #if STM32_USBH_USE_OTG2
      usbhMainLoop(&USBHD2);
    #endif
    #if STM32_USBH_USE_OTG1
      usbhMainLoop(&USBHD1);
    #endif
    chThdSleepMilliseconds(10);  // 10ms polling for USB events
  }
}
#endif



/* ============================================================================
 * NETWORK THREAD FUNCTIONS
 * ============================================================================ */

/* UDP Server Thread */
static THD_WORKING_AREA(waUdpServer, 512);
static THD_FUNCTION(UdpServerThread, arg) {
  (void)arg;
  chRegSetThreadName("udp_server");
  event_listener_t el;
  chEvtRegisterMask(&esDS5Ready, &el, EVT_DS5_READY);
  
  int sock;
  struct sockaddr_in server_addr;
  struct sockaddr_in bcast_addr;
  
  // Create UDP socket
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    chprintf((BaseSequentialStream *)&RTT_S0, "Failed to create UDP socket\n");
    return;
  }
  
  // Configure server address
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces
  server_addr.sin_port = htons(UDP_SERVER_PORT);

#if JOY_STREAM_ENABLE
  // Configure broadcast destination address
  memset(&bcast_addr, 0, sizeof(bcast_addr));
  bcast_addr.sin_family = AF_INET;
  bcast_addr.sin_addr.s_addr = INADDR_BROADCAST; // 255.255.255.255
  bcast_addr.sin_port = htons(JOY_STREAM_PORT);
#endif
  
  // Bind socket to address
  if (bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    chprintf((BaseSequentialStream *)&RTT_S0, "Failed to bind UDP socket to port %d\n", UDP_SERVER_PORT);
    close(sock);
    return;
  }

#if JOY_STREAM_ENABLE
  // Enable broadcast on this socket
  {
    int opt = 1;
    (void)setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
  }
#endif
  
  chprintf((BaseSequentialStream *)&RTT_S0, "UDP Server started on port %d\n", UDP_SERVER_PORT);
  
  while (true) {
    /* Wait for new DS5 data when idle; busy processing will naturally coalesce */
    chEvtWaitOne(EVT_DS5_READY);
    uint32_t g = ds5Generation;
    uint8_t idx = ds5PublishedIndex;
    /* Snapshot to stack to avoid partial reads if another publish happens */
    PS5Data snap;
    memcpy(&snap, &ds5Buf[idx], sizeof(PS5Data));
    (void)g; /* keep for future per-consumer versioning if needed */

    // Send raw PS5Data byte array
    sendto(sock, (const void*)&snap, sizeof(PS5Data), 0, (struct sockaddr*)&bcast_addr, sizeof(bcast_addr));
  }
}

/* ============================================================================
 * nRF24L01 THREAD FUNCTIONS
 * ============================================================================ */

/* nRF24 TX Broadcaster Thread - Sends latest DS5 snapshot over nRF24 in 32-byte chunks */
static THD_WORKING_AREA(waNRF24Tx, 128);
static THD_FUNCTION(NRF24TxThread, arg) {
  (void)arg;
  chRegSetThreadName("nrf24_tx");
  event_listener_t el;
  
  chEvtRegisterMask(&esDS5Ready, &el, EVT_DS5_READY);

  /* nRF24L01 TX destination address */
  uint8_t nrf_tx_addr[5] = {0x65, 0x64, 0x6F, 0x4E, 0x32}; // 0x32 0x4E 0x6F 0x64 0x65 = "2Node" !check endianness

  while (true) {
    chEvtWaitOne(EVT_DS5_READY);

    uint8_t idx = ds5PublishedIndex;
    PS5Data snap;
    convert_to_trimmed(&ds5Buf[idx], &snap);

    //! blocking call if set the use_ack to TRUE it will wait for the ack from the receiver
    if (nrf24l01_basic_send(nrf_tx_addr, (uint8_t *)&snap, sizeof(PS5Data), NRF24L01_BOOL_TRUE) != 0) {
      // Handle transmission error if needed
    }
  }
}

/* ============================================================================
 * NETWORK CALLBACK FUNCTIONS
 * ============================================================================ */

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
  /*
   * Creates the example threads.
   */
  /* Initialize latest-only DS5 event source */
  chEvtObjectInit(&esDS5Ready);
  
  /* Initialize NRF24L01 interrupt event source */
  chEvtObjectInit(&esNRF24Interrupt);

  uint8_t mac_address[6] = {0x02, 0x12, 0x13, 0x10, 0x15, 0x05};

  ip4_addr_t ip_addr, gateway_addr, netmask_addr;
  IP4_ADDR(&ip_addr, 192, 168, 0, 100);            // IP: 192.168.0.100
  IP4_ADDR(&gateway_addr, 192, 168, 0, 1);         // Gateway: 192.168.0.1
  IP4_ADDR(&netmask_addr, 255, 255, 255, 0);       // Netmask: 255.255.255.0

  // Set up the lwIP thread options
  lwipthread_opts_t lwipthread_opts = {
      .macaddress = mac_address,                   // MAC address array
      .address = ip_addr.addr,                     // IP address (32-bit)
      .netmask = netmask_addr.addr,                // Subnet mask (32-bit)
      .gateway = gateway_addr.addr,                // Gateway address (32-bit)
      .addrMode = NET_ADDRESS_STATIC,              // Address mode: STATIC, DHCP, or AUTO
      .ourHostName = "ds-eth-comm",                // Hostname (optional)
      .link_up_cb = myLinkUpCallback,              // Link up callback (optional)
      .link_down_cb = myLinkDownCallback           // Link down callback (optional)
  };

  usbhStart(&USBHD2);

  lwipInit(&lwipthread_opts);


  nrf24l01_basic_init(NRF24L01_TYPE_TX, interrupt_callback);
  palEnableLineEvent(LINE_SPI2_NRF24_IRQ, PAL_EVENT_MODE_FALLING_EDGE);
  palSetLineCallback(LINE_SPI2_NRF24_IRQ, nrf24l01_interface_gpio_interrupt_callback, NULL);
  
  
  

  uint8_t res = flashdb_init();
  chprintf((BaseSequentialStream *)&RTT_S0, "flashdb_init result: %d\n", res);
  char boot_count = 0;
  flashdb_get_kv_value("boot_count", &boot_count, sizeof(boot_count));
  boot_count++;
  flashdb_set_kv_value("boot_count", &boot_count, sizeof(boot_count));
  chprintf((BaseSequentialStream *)&RTT_S0, "Boot count: %d\n", boot_count);
  

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);
  chThdCreateStatic(waUdpServer, sizeof(waUdpServer), NORMALPRIO, UdpServerThread, NULL);
  chThdCreateStatic(waNRF24Tx, sizeof(waNRF24Tx), NORMALPRIO, NRF24TxThread, NULL);
  chThdCreateStatic(waNRF24Interrupt, sizeof(waNRF24Interrupt), NORMALPRIO+3, NRF24InterruptThread, NULL);
  chThdCreateStatic(waTestHID, sizeof(waTestHID), NORMALPRIO, ThreadTestHID, NULL);
  chThdCreateStatic(waUsbHost, sizeof(waUsbHost), NORMALPRIO+2, UsbHostThread, NULL);
  

  uint8_t nrf_tx_addr[5] = {0x65, 0x64, 0x6F, 0x4E, 0x32}; // 0x32 0x4E 0x6F 0x64 0x65 = "2Node" !check endianness
   PS5Data snap;
   // Initialize all fields for testing
   snap.hatValue[0] = 0x01;  // LX
   snap.hatValue[1] = 0x02;  // LY
   snap.hatValue[2] = 0x03;  // RX
   snap.hatValue[3] = 0x04;  // RY
   snap.trigger[0] = 0x00;   // L2
   snap.trigger[1] = 0x00;   // R2
   snap.sequence_number = 0x01;
   snap.btn.val = 0x000001;  // Set some button bits
   snap.reserved[0] = 0x00;
   snap.reserved[1] = 0x00;
   snap.reserved[2] = 0x00;
   snap.reserved[3] = 0x00;
   snap.reserved[4] = 0x00;
   snap.gyroX = 0x1234;
   snap.gyroZ = 0x5678;
   snap.gyroY = 0x9ABC;
   snap.accX = 0xDEF0;
   snap.accZ = 0x1111;
   snap.accY = 0x2222;
   snap.sensor_timestamp = 0x12345678;
  
   // Print hex dump of the snap structure
   print_ps5data_hex(&snap, "PS5Data Structure");
   
   res = nrf24l01_basic_send(nrf_tx_addr, (uint8_t *)&snap, sizeof(PS5Data), NRF24L01_BOOL_TRUE);
   chprintf((BaseSequentialStream *)&RTT_S0, "nrf24l01_basic_send result: %d\n", res);

  
  
  while (1) {
      thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
      "shell", NORMALPRIO + 1,
      shellThread, (void *)&shell_cfg1);
    chThdWait(shelltp); /* Waiting termination. */
    chThdSleepMilliseconds(500);
  }
}
