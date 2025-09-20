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

#define LWIP_PORT_INIT_IPADDR(addr)   IP4_ADDR((addr), 192,168,1,200)
#define LWIP_PORT_INIT_GW(addr)       IP4_ADDR((addr), 192,168,1,1)
#define LWIP_PORT_INIT_NETMASK(addr)  IP4_ADDR((addr), 255,255,255,0)


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


// UDP Server Configuration
#define UDP_SERVER_PORT    12345
#define UDP_BUFFER_SIZE    1024

/*
 * UDP Server Thread
 */
static THD_WORKING_AREA(waUdpServer, 2048);
static THD_FUNCTION(UdpServerThread, arg) {
  (void)arg;
  chRegSetThreadName("udp_server");
  
  int sock;
  struct sockaddr_in server_addr, client_addr;
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
  
  // Bind socket to address
  if (bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    chprintf((BaseSequentialStream *)&RTT_S0, "Failed to bind UDP socket to port %d\n", UDP_SERVER_PORT);
    close(sock);
    return;
  }
  
  chprintf((BaseSequentialStream *)&RTT_S0, "UDP Server started on port %d\n", UDP_SERVER_PORT);
  
  while (true) {
    // Wait for incoming data
    bytes_received = recvfrom(sock, buffer, UDP_BUFFER_SIZE - 1, 0, 
                             (struct sockaddr*)&client_addr, &client_len);
    
    if (bytes_received > 0) {
      buffer[bytes_received] = '\0';  // Null-terminate the string
      
      // Print received data and client info
      chprintf((BaseSequentialStream *)&RTT_S0, 
               "Received from %d.%d.%d.%d:%d: %s\n",
               (client_addr.sin_addr.s_addr >> 0) & 0xFF,
               (client_addr.sin_addr.s_addr >> 8) & 0xFF,
               (client_addr.sin_addr.s_addr >> 16) & 0xFF,
               (client_addr.sin_addr.s_addr >> 24) & 0xFF,
               ntohs(client_addr.sin_port),
               buffer);
    }
    else if (bytes_received < 0) {
      chprintf((BaseSequentialStream *)&RTT_S0, "Error receiving UDP data\n");
      chThdSleepMilliseconds(100);
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
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);
  chThdCreateStatic(waUdpServer, sizeof(waUdpServer), NORMALPRIO, UdpServerThread, NULL);

  while (1) {
    chThdSleepMilliseconds(500);
  }
}
