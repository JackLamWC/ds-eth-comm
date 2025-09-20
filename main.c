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

#define UDP_SERVER_PORT 1234
#define UDP_BUFFER_SIZE 1024

// Network configuration - DHCP is now enabled
// The device will automatically get IP address from your router
// These values are only used for MAC address and hostname

// MAC Address configuration
// Option 1: Use default MAC address (C2:AF:51:03:CF:46) by setting .macaddress = NULL
// Option 2: Use custom MAC address by uncommenting and modifying the values below
// #define DEVICE_MAC_0       0xC2  // Default: 0xC2
// #define DEVICE_MAC_1       0xAF  // Default: 0xAF  
// #define DEVICE_MAC_2       0x51  // Default: 0x51
// #define DEVICE_MAC_3       0x03  // Default: 0x03
// #define DEVICE_MAC_4       0xCF  // Default: 0xCF
// #define DEVICE_MAC_5       0x46  // Default: 0x46

// Network mode selection
#define USE_DHCP            1    // Set to 1 for DHCP, 0 for static IP

// Fallback definitions in case constants are not available
#ifndef NET_ADDRESS_DHCP
#define NET_ADDRESS_DHCP    1
#endif
#ifndef NET_ADDRESS_STATIC
#define NET_ADDRESS_STATIC  2
#endif

// Static IP configuration (not used when DHCP is enabled)
// Uncomment and change .addrMode to NET_ADDRESS_STATIC to use static IP
#define DEVICE_IP_A        192
#define DEVICE_IP_B        168
#define DEVICE_IP_C        0
#define DEVICE_IP_D        100

#define GATEWAY_A          192
#define GATEWAY_B          168
#define GATEWAY_C          0
#define GATEWAY_D          1

#define NETMASK_A          255
#define NETMASK_B          255
#define NETMASK_C          255
#define NETMASK_D          0






/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(50);
    palClearLine(LINE_LED_RED);
    chThdSleepMilliseconds(200);
    palSetLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(50);
    palSetLine(LINE_LED_RED);
    chThdSleepMilliseconds(200);
  }
}

static THD_WORKING_AREA(waUdpServer, 2048);
static THD_FUNCTION(UdpServerThread, arg) {
    int sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char buffer[UDP_BUFFER_SIZE];
    int bytes_received;
    
    (void)arg;
    chRegSetThreadName("udp_server");
    
    // Create UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        chprintf((BaseSequentialStream*)&RTT_S0, "Failed to create socket\n");
        return;
    }
    
    // Configure server address
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(UDP_SERVER_PORT);
    
    // Bind socket to port
    if (bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        chprintf((BaseSequentialStream*)&RTT_S0, "Failed to bind socket to port %d\n", UDP_SERVER_PORT);
        close(sock);
        return;
    }
    
    chprintf((BaseSequentialStream*)&RTT_S0, "UDP server started on port %d\n", UDP_SERVER_PORT);
    chprintf((BaseSequentialStream*)&RTT_S0, "Binding to: %s:%d\n", 
             inet_ntoa(server_addr.sin_addr), ntohs(server_addr.sin_port));
    
    // Get the actual IP address this server is listening on
    struct sockaddr_in actual_addr;
    socklen_t actual_len = sizeof(actual_addr);
    if (getsockname(sock, (struct sockaddr*)&actual_addr, &actual_len) == 0) {
        chprintf((BaseSequentialStream*)&RTT_S0, "Server listening on: %s:%d\n", 
                 inet_ntoa(actual_addr.sin_addr), ntohs(actual_addr.sin_port));
    }
    
    chprintf((BaseSequentialStream*)&RTT_S0, "UDP server thread is running...\n");
    
    int loop_count = 0;
    while (true) {
        loop_count++;
        
        // Print heartbeat every 1000 loops (about 10 seconds)
        if (loop_count % 1000 == 0) {
            chprintf((BaseSequentialStream*)&RTT_S0, "UDP server heartbeat - waiting for packets...\n");
        }
        // Receive data
        bytes_received = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                                 (struct sockaddr*)&client_addr, &client_len);
        
        if (bytes_received > 0) {
            buffer[bytes_received] = '\0';
            chprintf((BaseSequentialStream*)&RTT_S0, "Received %d bytes from %s:%d: %s\n", 
                   bytes_received, 
                   inet_ntoa(client_addr.sin_addr), 
                   ntohs(client_addr.sin_port),
                   buffer);
            
            // Echo back the data
            sendto(sock, buffer, bytes_received, 0,
                   (struct sockaddr*)&client_addr, client_len);
        } else if (bytes_received < 0) {
            // Error occurred
            chprintf((BaseSequentialStream*)&RTT_S0, "UDP recv error: %d\n", bytes_received);
        }
        
        // Small delay to prevent busy waiting
        chThdSleepMilliseconds(10);
    }
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


  
  
  // Configure network parameters
  static lwipthread_opts_t lwip_opts = {
    .macaddress = NULL,                       // Use default MAC address (C2:AF:51:03:CF:46)
#if USE_DHCP
    .address = 0,                             // Not used for DHCP
    .netmask = 0,                             // Not used for DHCP
    .gateway = 0,                             // Not used for DHCP
    .addrMode = NET_ADDRESS_DHCP,             // DHCP mode - get IP automatically
#else
    .address = PP_HTONL(LWIP_MAKEU32(DEVICE_IP_A, DEVICE_IP_B, DEVICE_IP_C, DEVICE_IP_D)),    // Static IP
    .netmask = PP_HTONL(LWIP_MAKEU32(NETMASK_A, NETMASK_B, NETMASK_C, NETMASK_D)),    // Subnet mask  
    .gateway = PP_HTONL(LWIP_MAKEU32(GATEWAY_A, GATEWAY_B, GATEWAY_C, GATEWAY_D)),      // Gateway
    .addrMode = NET_ADDRESS_STATIC,            // Static IP mode
#endif
#if LWIP_NETIF_HOSTNAME
    .ourHostName = "STM32_Device",            // Hostname (optional)
#endif
    .link_up_cb = NULL,                       // Link up callback (optional)
    .link_down_cb = NULL                      // Link down callback (optional)
  };
  
  lwipInit(&lwip_opts);
  /*
   * Activates the serial driver 3 using the driver default configuration.
   */
  // sdStart(&SD3, NULL);
  
  /*
   * Wait for network initialization
   */
#if USE_DHCP
  chprintf((BaseSequentialStream*)&RTT_S0, "Starting DHCP client...\n");
#else
  chprintf((BaseSequentialStream*)&RTT_S0, "Using static IP configuration...\n");
#endif
  chThdSleepMilliseconds(1000);
  
  // Get the default network interface
  struct netif *netif = netif_default;
  if (netif != NULL) {
    chprintf((BaseSequentialStream*)&RTT_S0, "Network interface found\n");
    chprintf((BaseSequentialStream*)&RTT_S0, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
             netif->hwaddr[0], netif->hwaddr[1], netif->hwaddr[2],
             netif->hwaddr[3], netif->hwaddr[4], netif->hwaddr[5]);
    
    // Check link status
    if (netif_is_link_up(netif)) {
      chprintf((BaseSequentialStream*)&RTT_S0, "Link is UP\n");
    } else {
      chprintf((BaseSequentialStream*)&RTT_S0, "Link is DOWN - check cable connection\n");
    }
    
    // Wait a bit for link to stabilize
    chprintf((BaseSequentialStream*)&RTT_S0, "Waiting for link to stabilize...\n");
    chThdSleepMilliseconds(5000);
    
    // Check link status again
    if (netif_is_link_up(netif)) {
      chprintf((BaseSequentialStream*)&RTT_S0, "Link is UP after wait\n");
    } else {
      chprintf((BaseSequentialStream*)&RTT_S0, "Link is still DOWN after wait\n");
    }
    
#if USE_DHCP
    // Only attempt DHCP if link is up
    if (netif_is_link_up(netif)) {
      chprintf((BaseSequentialStream*)&RTT_S0, "Link is UP - starting DHCP...\n");
      // Wait for DHCP to complete (up to 10 seconds)
      int dhcp_timeout = 10; // 10 seconds (100 * 100ms)
      while (dhcp_timeout > 0 && !netif_is_up(netif)) {
      chThdSleepMilliseconds(1000);
      dhcp_timeout--;
      if (dhcp_timeout % 10 == 0) { // Print every second
        chprintf((BaseSequentialStream*)&RTT_S0, "Waiting for DHCP... (%d seconds)\n", dhcp_timeout/10);
        // Check current IP status during wait
        const ip4_addr_t *current_ip = netif_ip4_addr(netif);
        chprintf((BaseSequentialStream*)&RTT_S0, "Current IP: %s\n", ip4addr_ntoa(current_ip));
      }
    }
    
    if (netif_is_up(netif)) {
      chprintf((BaseSequentialStream*)&RTT_S0, "DHCP successful!\n");
      // Wait a bit more for IP to be fully assigned
      chThdSleepMilliseconds(1000);
      
      // Check if we actually got valid IP configuration
      const ip4_addr_t *ip = netif_ip4_addr(netif);
      const ip4_addr_t *mask = netif_ip4_netmask(netif);
      const ip4_addr_t *gw = netif_ip4_gw(netif);
      
      if (ip4_addr_get_u32(ip) == 0) {
        chprintf((BaseSequentialStream*)&RTT_S0, "WARNING: No valid IP address received!\n");
        chprintf((BaseSequentialStream*)&RTT_S0, "DHCP completed but no IP assigned\n");
        chprintf((BaseSequentialStream*)&RTT_S0, "Debug: IP=0x%08X, Mask=0x%08X, GW=0x%08X\n", 
                 ip4_addr_get_u32(ip), ip4_addr_get_u32(mask), ip4_addr_get_u32(gw));
      } else {
        chprintf((BaseSequentialStream*)&RTT_S0, "Valid IP configuration received\n");
        
        // Additional network status check
        chprintf((BaseSequentialStream*)&RTT_S0, "Network Status Check:\n");
        chprintf((BaseSequentialStream*)&RTT_S0, "  Link UP: %s\n", netif_is_link_up(netif) ? "YES" : "NO");
        chprintf((BaseSequentialStream*)&RTT_S0, "  Interface UP: %s\n", netif_is_up(netif) ? "YES" : "NO");
        chprintf((BaseSequentialStream*)&RTT_S0, "  IP: %s\n", ip4addr_ntoa(ip));
        chprintf((BaseSequentialStream*)&RTT_S0, "  Mask: %s\n", ip4addr_ntoa(mask));
        chprintf((BaseSequentialStream*)&RTT_S0, "  Gateway: %s\n", ip4addr_ntoa(gw));
      }
      } else {
        chprintf((BaseSequentialStream*)&RTT_S0, "DHCP failed or timeout\n");
      }
    } else {
      chprintf((BaseSequentialStream*)&RTT_S0, "Link is DOWN - skipping DHCP\n");
    }
#else
    // For static IP, just wait a bit for interface to come up
    chThdSleepMilliseconds(1000);
#endif

    if (netif_is_up(netif)) {
      chprintf((BaseSequentialStream*)&RTT_S0, "Network interface is UP\n");
      chprintf((BaseSequentialStream*)&RTT_S0, "IP Address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
      chprintf((BaseSequentialStream*)&RTT_S0, "Netmask: %s\n", ip4addr_ntoa(netif_ip4_netmask(netif)));
      chprintf((BaseSequentialStream*)&RTT_S0, "Gateway: %s\n", ip4addr_ntoa(netif_ip4_gw(netif)));
    } else {
      chprintf((BaseSequentialStream*)&RTT_S0, "Network interface is DOWN\n");
    }
  } else {
    chprintf((BaseSequentialStream*)&RTT_S0, "Network interface not available\n");
  }

  /*
   * Creates the example threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);
  chThdCreateStatic(waUdpServer, sizeof(waUdpServer), NORMALPRIO+2, UdpServerThread, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  int status_count = 0;
  while (1) {
    // Check network status every 10 seconds
    if (status_count % 20 == 0) { // Every 10 seconds (20 * 500ms)
      if (netif != NULL) {
        chprintf((BaseSequentialStream*)&RTT_S0, "Network Status: Link=%s, Up=%s, IP=%s\n",
                 netif_is_link_up(netif) ? "UP" : "DOWN",
                 netif_is_up(netif) ? "UP" : "DOWN",
                 ip4addr_ntoa(netif_ip4_addr(netif)));
      }
    }
    status_count++;
    chThdSleepMilliseconds(500);
  }
}
