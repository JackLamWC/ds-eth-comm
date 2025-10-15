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

#include "ch.h"
#include "hal.h"
#include "lwipthread.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/netif.h"
#include "chprintf.h"
#include "SEGGER_RTT_Channel.h"
#include "SEGGER_RTT.h"
#include "usbh/debug.h"
#include "w25qxx_interface.h"

static w25qxx_handle_t w25qxx_handle;

void w25qxx_interface_debug_print(const char *const fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  chvprintf((BaseSequentialStream *)&RTT_S0, fmt, ap);
  va_end(ap);
}

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
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

#if HAL_USBH_USE_HID
#include "usbh/dev/hid.h"
#include "chprintf.h"

// PlayStation Controller HID Report Structure
typedef struct {
    // Report ID (always 0x01)
    uint8_t report_id;
    
    // Analog sticks (0x00-0xFF, neutral ~0x80)
    uint8_t left_stick_x;    // Left stick X axis
    uint8_t left_stick_y;    // Left stick Y axis  
    uint8_t right_stick_x;   // Right stick X axis
    uint8_t right_stick_y;   // Right stick Y axis
    
    // Triggers (0x00-0xFF, neutral 0x00)
    uint8_t l2_trigger;      // L2 trigger axis
    uint8_t r2_trigger;      // R2 trigger axis
    
    // Vendor data
    uint8_t vendor_data1;
    
    // D-pad (4 bits) + Face buttons (4 bits)
    union {
        uint8_t raw;
        struct {
            uint8_t dpad : 4;        // D-pad direction
            uint8_t face_buttons : 4; // Square, Cross, Circle, Triangle
        };
    } dpad_face;
    
    // Shoulder and system buttons
    union {
        uint8_t raw;
        struct {
            uint8_t l1 : 1;      // L1 button
            uint8_t r1 : 1;       // R1 button
            uint8_t l2 : 1;       // L2 button
            uint8_t r2 : 1;       // R2 button
            uint8_t create : 1;   // Create button
            uint8_t options : 1;  // Options button
            uint8_t l3 : 1;       // L3 button
            uint8_t r3 : 1;       // R3 button
        };
    } shoulder_buttons;
    
    // System buttons and vendor data
    union {
        uint8_t raw;
        struct {
            uint8_t ps_button : 1;    // PS button
            uint8_t touchpad : 1;     // Touchpad button
            uint8_t mute : 1;         // Mute button
            uint8_t vendor_bits : 5;  // Vendor defined bits
        };
    } system_buttons;
    
    // Additional vendor data (52 bytes)
    uint8_t vendor_data[52];
    
} ps_controller_report_t;

// D-pad direction constants
typedef enum {
    DPAD_NEUTRAL = 0x8,
    DPAD_NORTH = 0x0,
    DPAD_NORTHEAST = 0x1,
    DPAD_EAST = 0x2,
    DPAD_SOUTHEAST = 0x3,
    DPAD_SOUTH = 0x4,
    DPAD_SOUTHWEST = 0x5,
    DPAD_WEST = 0x6,
    DPAD_NORTHWEST = 0x7
} dpad_direction_t;

// Face button constants
typedef enum {
    FACE_SQUARE = 0x01,
    FACE_CROSS = 0x02,
    FACE_CIRCLE = 0x04,
    FACE_TRIANGLE = 0x08
} face_button_t;

typedef enum {
    PAD_DPAD_NEUTRAL = 0,
    PAD_DPAD_NORTH   = 1,
    PAD_DPAD_EAST    = 2,
    PAD_DPAD_SOUTH   = 3,
    PAD_DPAD_WEST    = 4,
} pad_dpad_dir_t;

typedef struct {
    uint32_t ts_ms;   /* timestamp in milliseconds */
    uint8_t lx, ly;   /* 0..255 */
    uint8_t rx, ry;   /* 0..255 */
    uint8_t l2, r2;   /* 0..255 */
    pad_dpad_dir_t dpad;
    /* Digital buttons */
    uint8_t square, cross, circle, triangle;
    uint8_t l1, r1, l2_btn, r2_btn;
    uint8_t l3, r3;
    uint8_t create, options;
    uint8_t ps, touchpad, mute;
    uint32_t buttons_mask; /* compressed bitmask incl. dpad U/R/D/L */
} pad_state_t;

/* Inter-thread communication: pad_state_t mailbox over a memory pool */
#define JOY_POOL_SIZE     8
#define JOY_MB_CAPACITY   8

static memory_pool_t joyPool;
static pad_state_t joyPoolBuf[JOY_POOL_SIZE];
static mailbox_t joyMB;
static msg_t joyMBSlots[JOY_MB_CAPACITY];
static pad_state_t lastJoy;
static mutex_t joyMtx;


static inline float _norm_stick_u8(uint8_t v) {
    return ((float)((int)v - 128)) / 127.0f; /* ~[-1..1] */
}

static inline float _norm_trigger_u8(uint8_t v) {
    return (2.0f * ((float)v / 255.0f)) - 1.0f; /* [-1..1], released=+1, pressed=-1 */
}

/* Map HID hat to ROS-style D-pad axes: left=+1, right=-1, up=+1, down=-1, neutral=0 */
static inline void _map_hat_to_axes(uint8_t hat, float *lr, float *ud) {
    float x = 0.0f, y = 0.0f;
    switch (hat) {
        case 0x0: y = +1.0f; break;                 /* N */
        case 0x1: y = +1.0f; x = -1.0f; break;      /* NE (right -> -1) */
        case 0x2: x = -1.0f; break;                 /* E (right -> -1) */
        case 0x3: y = -1.0f; x = -1.0f; break;      /* SE */
        case 0x4: y = -1.0f; break;                 /* S */
        case 0x5: y = -1.0f; x = +1.0f; break;      /* SW (left -> +1) */
        case 0x6: x = +1.0f; break;                 /* W (left -> +1) */
        case 0x7: y = +1.0f; x = +1.0f; break;      /* NW */
        default: /* 0x8 neutral */ break;
    }
    *lr = x;  /* axes[6] */
    *ud = y;  /* axes[7] */
}

static inline pad_dpad_dir_t hat_to_cardinal(uint8_t hat) {
    switch (hat) { /* 0=N,1=NE,2=E,3=SE,4=S,5=SW,6=W,7=NW,8=neutral */
        case 0: return PAD_DPAD_NORTH;
        case 1: return PAD_DPAD_NORTH; /* NE -> N */
        case 2: return PAD_DPAD_EAST;
        case 3: return PAD_DPAD_SOUTH; /* SE -> S */
        case 4: return PAD_DPAD_SOUTH;
        case 5: return PAD_DPAD_SOUTH; /* SW -> S */
        case 6: return PAD_DPAD_WEST;
        case 7: return PAD_DPAD_NORTH; /* NW -> N */
        default: return PAD_DPAD_NEUTRAL;
    }
}

static inline void ros_joy_dpad_bools_hat(uint8_t hat, int *btn_up, int *btn_down, int *btn_left, int *btn_right) {
  /* hat values: 0=N,1=NE,2=E,3=SE,4=S,5=SW,6=W,7=NW,8=neutral */
  *btn_up = (hat == 0 || hat == 1 || hat == 7) ? 1 : 0;
  *btn_right = (hat == 1 || hat == 2 || hat == 3) ? 1 : 0;
  *btn_down = (hat == 3 || hat == 4 || hat == 5) ? 1 : 0;
  *btn_left = (hat == 5 || hat == 6 || hat == 7) ? 1 : 0;
}

static inline void ps_to_pad_state(const ps_controller_report_t *r, pad_state_t *out) {
    out->ts_ms = TIME_I2MS(chVTGetSystemTimeX());
    out->lx = r->left_stick_x;
    out->ly = r->left_stick_y;
    out->rx = r->right_stick_x;
    out->ry = r->right_stick_y;
    out->l2 = r->l2_trigger;
    out->r2 = r->r2_trigger;
    out->dpad = hat_to_cardinal((uint8_t)r->dpad_face.dpad);
    out->square   = (r->dpad_face.face_buttons & FACE_SQUARE)   ? 1 : 0;
    out->cross    = (r->dpad_face.face_buttons & FACE_CROSS)    ? 1 : 0;
    out->circle   = (r->dpad_face.face_buttons & FACE_CIRCLE)   ? 1 : 0;
    out->triangle = (r->dpad_face.face_buttons & FACE_TRIANGLE) ? 1 : 0;
    out->l1 = r->shoulder_buttons.l1 ? 1 : 0;
    out->r1 = r->shoulder_buttons.r1 ? 1 : 0;
    out->l2_btn = r->shoulder_buttons.l2 ? 1 : 0;
    out->r2_btn = r->shoulder_buttons.r2 ? 1 : 0;
    out->l3 = r->shoulder_buttons.l3 ? 1 : 0;
    out->r3 = r->shoulder_buttons.r3 ? 1 : 0;
    out->create  = r->shoulder_buttons.create  ? 1 : 0;
    out->options = r->shoulder_buttons.options ? 1 : 0;
    out->ps       = r->system_buttons.ps_button ? 1 : 0;
    out->touchpad = r->system_buttons.touchpad  ? 1 : 0;
    out->mute     = r->system_buttons.mute      ? 1 : 0;

    /* Build compressed buttons mask */
    uint32_t m = 0;
#define SETB(bit, val) do { if (val) m |= (1u << (bit)); } while (0)
    /* 0..14 regular buttons */
    SETB(0, out->square);
    SETB(1, out->cross);
    SETB(2, out->circle);
    SETB(3, out->triangle);
    SETB(4, out->l1);
    SETB(5, out->r1);
    SETB(6, out->l2_btn);
    SETB(7, out->r2_btn);
    SETB(8, out->l3);
    SETB(9, out->r3);
    SETB(10, out->create);
    SETB(11, out->options);
    SETB(12, out->ps);
    SETB(13, out->touchpad);
    SETB(14, out->mute);
    /* 15..18 D-pad U/R/D/L from hat */
    int u=0,d=0,l=0,rgt=0;
    ros_joy_dpad_bools_hat((uint8_t)r->dpad_face.dpad, &u, &d, &l, &rgt);
    SETB(15, u);
    SETB(16, rgt);
    SETB(17, d);
    SETB(18, l);
#undef SETB
    out->buttons_mask = m;
}

static inline const char *pad_dpad_str(pad_dpad_dir_t d) {
    switch (d) {
        case PAD_DPAD_NORTH: return "N";
        case PAD_DPAD_EAST:  return "E";
        case PAD_DPAD_SOUTH: return "S";
        case PAD_DPAD_WEST:  return "W";
        default:             return "NEUTRAL";
    }
}


static THD_WORKING_AREA(waTestHID, 1024);

static void _hid_report_callback(USBHHIDDriver *hidp, uint16_t len) {
    uint8_t *report = (uint8_t *)hidp->config->report_buffer;

    if (hidp->type == USBHHID_DEVTYPE_BOOT_MOUSE) {
        chprintf((BaseSequentialStream *)&RTT_S0, "Mouse report: buttons=%02x, Dx=%d, Dy=%d\n",
                report[0],
                (int8_t)report[1],
                (int8_t)report[2]);
    } else if (hidp->type == USBHHID_DEVTYPE_BOOT_KEYBOARD) {
        // chprintf((BaseSequentialStream *)&RTT_S0, "Keyboard report: modifier=%02x, keys=%02x %02x %02x %02x %02x %02x\n",
        //         report[0],
        //         report[2],
        //         report[3],
        //         report[4],
        //         report[5],
        //         report[6],
        //         report[7]);
    } else {
        // Check if this is a PlayStation controller report
        if (len >= sizeof(ps_controller_report_t) && report[0] == 0x01) {
            // Cast the raw data to our struct
            ps_controller_report_t *controller = (ps_controller_report_t *)report;
            
            // Parse analog sticks (convert to signed values)
            int16_t left_x = (int16_t)controller->left_stick_x - 128;
            int16_t left_y = (int16_t)controller->left_stick_y - 128;
            int16_t right_x = (int16_t)controller->right_stick_x - 128;
            int16_t right_y = (int16_t)controller->right_stick_y - 128;
            
            // Parse D-pad
            dpad_direction_t dpad = (dpad_direction_t)controller->dpad_face.dpad;
            
            // Parse face buttons
            bool square = (controller->dpad_face.face_buttons & FACE_SQUARE) != 0;
            bool cross = (controller->dpad_face.face_buttons & FACE_CROSS) != 0;
            bool circle = (controller->dpad_face.face_buttons & FACE_CIRCLE) != 0;
            bool triangle = (controller->dpad_face.face_buttons & FACE_TRIANGLE) != 0;
            
            // Build ROS-like joy output
            pad_state_t *joy = (pad_state_t *)chPoolAlloc(&joyPool);
            if (joy == NULL) {
                // Pool exhausted, drop this frame
                return;
            }
            ps_to_pad_state(controller, joy);

            // Post to mailbox (try, drop if full)
            (void)chMBPostTimeout(&joyMB, (msg_t)joy, TIME_IMMEDIATE);
        } else {
            // Handle other generic HID devices
            chprintf((BaseSequentialStream *)&RTT_S0, "Generic HID report, %d bytes\n", len);
            chprintf((BaseSequentialStream *)&RTT_S0, "Report: ");
            for (int i = 0; i < (len < 16 ? len : 16); i++) {
                chprintf((BaseSequentialStream *)&RTT_S0, "%02x ", report[i]);
            }
            chprintf((BaseSequentialStream *)&RTT_S0, "\n");
        }
    }
}

static USBH_DEFINE_BUFFER(uint8_t report[HAL_USBHHID_MAX_INSTANCES][64]);
static USBHHIDConfig hidcfg[HAL_USBHHID_MAX_INSTANCES];

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
#endif



// UDP Server Configuration
#define UDP_SERVER_PORT    12345
#define UDP_BUFFER_SIZE    1024
// Optional JSON streaming (broadcast) so clients can just listen
#define JOY_STREAM_ENABLE  1
#define JOY_STREAM_PORT    12346

/*
 * UDP Server Thread
 */
static THD_WORKING_AREA(waUdpServer, 2048);
static THD_FUNCTION(UdpServerThread, arg) {
  (void)arg;
  chRegSetThreadName("udp_server");
  
  int sock;
  struct sockaddr_in server_addr, client_addr;
  struct sockaddr_in bcast_addr;
  socklen_t client_len = sizeof(client_addr);
  uint8_t buffer[UDP_BUFFER_SIZE];
  int bytes_received;
  
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
    // Non-blocking HID->UDP forwarding: poll mailbox with timeout
    msg_t m;
    msg_t mbres = chMBFetchTimeout(&joyMB, &m, TIME_MS2I(50));

    if (mbres == MSG_OK) {
      pad_state_t *joy = (pad_state_t *)m;
      // Optionally broadcast JSON snapshot for listeners
#if JOY_STREAM_ENABLE
      char jout[512];
      int jn = chsnprintf(jout, sizeof(jout),
        "{\"timestamp_ms\":%u,\"axes\":[%u,%u,%u,%u,%u,%u],\"buttons\":%u}",
        joy->ts_ms, joy->lx, joy->ly, joy->rx, joy->ry, joy->l2, joy->r2, joy->buttons_mask);
      chprintf((BaseSequentialStream *)&RTT_S0, "JSON: %s\n", jout);
      sendto(sock, jout, (size_t)jn, 0, (struct sockaddr*)&bcast_addr, sizeof(bcast_addr));
#endif
      // Return buffer to pool
      chPoolFree(&joyPool, joy);
    }
  }
}

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


void spi_error_cb(SPIDriver *spip) {
  chprintf((BaseSequentialStream *)&RTT_S0, "SPI error\n");
}


static const SPIConfig spi_config = {
  .circular = false,
  .ssline = LINE_SPI_FLASH_CS,
  .slave = false,
  .data_cb = NULL,
  .error_cb = spi_error_cb,
  .cfg1 = SPI_CFG1_MBR_1 | SPI_CFG1_MBR_2 | SPI_CFG1_DSIZE_8BITS,
  .cfg2 = SPI_CFG2_CPHA | SPI_CFG2_CPOL
};

static w25qxx_handle_t w25qxx_handle;

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

  DRIVER_W25QXX_LINK_INIT(&w25qxx_handle, &spi_config);
  DRIVER_W25QXX_LINK_SPI_QSPI_INIT(&w25qxx_handle, w25qxx_interface_spi_qspi_init);
  DRIVER_W25QXX_LINK_SPI_QSPI_DEINIT(&w25qxx_handle, w25qxx_interface_spi_qspi_deinit);
  DRIVER_W25QXX_LINK_SPI_QSPI_WRITE_READ(&w25qxx_handle, w25qxx_interface_spi_qspi_write_read);
  DRIVER_W25QXX_LINK_DELAY_MS(&w25qxx_handle, w25qxx_interface_delay_ms);
  DRIVER_W25QXX_LINK_DELAY_US(&w25qxx_handle, w25qxx_interface_delay_us);
  DRIVER_W25QXX_LINK_DEBUG_PRINT(&w25qxx_handle, w25qxx_interface_debug_print);
  w25qxx_set_interface(&w25qxx_handle, W25QXX_INTERFACE_SPI);
  w25qxx_set_type(&w25qxx_handle, W25Q64);
  w25qxx_init(&w25qxx_handle);
  
  // Initialize SPI
  // spiStart(&SPID1, &spi_config);

  // chThdSleepMilliseconds(1000);
  
  // // Test SPI communication
  // static uint8_t tx[4] = {0x9F, 0x00, 0x00, 0x00};  // JEDEC ID command
  // static uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00};
  
  // chprintf((BaseSequentialStream *)&RTT_S0, "Testing SPI communication...\n");
  
  // spiSelect(&SPID1);
  // msg_t res = spiExchange(&SPID1, 4, tx, rx);
  // spiUnselect(&SPID1);
  
  // if (res == MSG_OK) {
  //   chprintf((BaseSequentialStream *)&RTT_S0, "SPI Exchange OK\n");
  //   chprintf((BaseSequentialStream *)&RTT_S0, "TX: 0x%02x 0x%02x 0x%02x 0x%02x\n", tx[0], tx[1], tx[2], tx[3]);
  //   chprintf((BaseSequentialStream *)&RTT_S0, "RX: 0x%02x 0x%02x 0x%02x 0x%02x\n", rx[0], rx[1], rx[2], rx[3]);
    
  //   // Check if we got valid JEDEC ID response
  //   if (rx[0] != 0x00 || rx[1] != 0x00 || rx[2] != 0x00) {
  //     chprintf((BaseSequentialStream *)&RTT_S0, "Flash JEDEC ID: 0x%02x%02x%02x\n", rx[0], rx[1], rx[2]);
  //   } else {
  //     chprintf((BaseSequentialStream *)&RTT_S0, "No response from flash - check connections\n");
  //   }
  // } else {
  //   chprintf((BaseSequentialStream *)&RTT_S0, "SPI exchange failed: %d\n", res);
  // }
  

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

  lwipInit(&lwipthread_opts);
  /*
   * Creates the example threads.
   */
  /* Initialize pad_state_t pool and mailbox */
  chPoolObjectInit(&joyPool, sizeof(pad_state_t), NULL);
  chPoolLoadArray(&joyPool, joyPoolBuf, JOY_POOL_SIZE);
  chMBObjectInit(&joyMB, joyMBSlots, JOY_MB_CAPACITY);
  chMtxObjectInit(&joyMtx);

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);
  chThdCreateStatic(waUdpServer, sizeof(waUdpServer), NORMALPRIO, UdpServerThread, NULL);
  chThdCreateStatic(waTestHID, sizeof(waTestHID), NORMALPRIO, ThreadTestHID, NULL);

  #if STM32_USBH_USE_OTG1
      usbhStart(&USBHD1);
      chprintf((BaseSequentialStream *)&RTT_S0, "USBH OTG2 start requested\n");
  #endif
  #if STM32_USBH_USE_OTG2
      usbhStart(&USBHD2);
      chprintf((BaseSequentialStream *)&RTT_S0, "USBH OTG2 started");
  #endif

  while (1) {
    #if STM32_USBH_USE_OTG2
      usbhMainLoop(&USBHD2);
    #endif
    #if STM32_USBH_USE_OTG1
      usbhMainLoop(&USBHD1);
    #endif
    chThdSleepMilliseconds(1000);
  }
}
