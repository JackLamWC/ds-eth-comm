# STM32 Controller Communication System - Project Tasks

## Project Overview
This document outlines the implementation tasks for an STM32-based controller communication system that includes:
- NRF24L01+ wireless data transmission
- OLED display monitoring
- FlashDB settings storage using W25Q64 flash

## Current Project Status
- **Base System**: ChibiOS RTOS with STM32H750
- **Existing Features**: 
  - USB HID controller support (PlayStation controller)
  - UDP server for data transmission
  - W25Q64 flash memory support
  - Ethernet connectivity
  - Shell interface

---

## Task 1: NRF24L01+ Data Transmission 📡

### 1.1 Hardware Setup
- **Module**: NRF24L01+ 2.4GHz transceiver
- **Interface**: SPI communication
- **Power**: 3.3V operation

### 1.2 Pin Configuration
Add to `board/board.h`:
```c
// NRF24L01+ Pin definitions
#define LINE_NRF24_CE          PAL_LINE(GPIOA, 8)    // Chip Enable
#define LINE_NRF24_CS          PAL_LINE(GPIOA, 9)    // Chip Select
#define LINE_NRF24_IRQ         PAL_LINE(GPIOA, 10)   // Interrupt Request
// SPI pins already defined for W25Q64 (MOSI, MISO, SCK)
```

### 1.3 SPI Configuration
- **SPI Interface**: Use existing SPI1 or add SPI2
- **Clock Speed**: 10MHz maximum for NRF24L01+
- **Mode**: SPI_MODE_0 (CPOL=0, CPHA=0)

### 1.4 Driver Implementation
Create `src/nrf24l01/nrf24l01.h` and `src/nrf24l01/nrf24l01.c`:

**Key Functions:**
- `nrf24l01_init()` - Initialize module
- `nrf24l01_write_register()` - Write to register
- `nrf24l01_read_register()` - Read from register
- `nrf24l01_set_tx_mode()` - Configure for transmission
- `nrf24l01_set_rx_mode()` - Configure for reception
- `nrf24l01_send_data()` - Send data packet
- `nrf24l01_receive_data()` - Receive data packet

### 1.5 Integration with UDP Server
Modify `UdpServerThread` to also send data via NRF24L01+:
- Add NRF24L01+ transmission alongside UDP
- Create separate thread for NRF24L01+ operations
- Implement data queuing for reliable transmission

---

## Task 2: OLED Display Monitor 📺

### 2.1 Hardware Setup
- **Display**: SSD1306 128x64 OLED (I2C or SPI)
- **Interface**: I2C recommended (simpler wiring)
- **Power**: 3.3V operation

### 2.2 Pin Configuration
Add to `board/board.h`:
```c
// OLED Display Pin definitions (I2C)
#define LINE_OLED_SDA          PAL_LINE(GPIOB, 7)    // I2C1 SDA
#define LINE_OLED_SCL          PAL_LINE(GPIOB, 6)    // I2C1 SCL
// OR for SPI interface:
// #define LINE_OLED_CS           PAL_LINE(GPIOA, 11)   // Chip Select
// #define LINE_OLED_DC           PAL_LINE(GPIOA, 12)   // Data/Command
// #define LINE_OLED_RST          PAL_LINE(GPIOA, 13)   // Reset
```

### 2.3 I2C Configuration
- **Interface**: I2C1
- **Clock Speed**: 400kHz (fast mode)
- **Address**: 0x3C (default for SSD1306)

### 2.4 Driver Implementation
Create `src/oled/oled.h` and `src/oled/oled.c`:

**Key Functions:**
- `oled_init()` - Initialize display
- `oled_clear()` - Clear screen
- `oled_set_pixel()` - Set pixel at (x,y)
- `oled_draw_char()` - Draw character
- `oled_draw_string()` - Draw text string
- `oled_draw_line()` - Draw line
- `oled_draw_rect()` - Draw rectangle
- `oled_update()` - Update display buffer

### 2.5 Display Content
Create `src/oled/oled_display.h` and `src/oled/oled_display.c`:

**Display Layout:**
```
┌─────────────────────────┐
│ Controller Status       │
├─────────────────────────┤
│ L: 128  R: 128          │
│ X: 128  Y: 128          │
├─────────────────────────┤
│ L2: 128  R2: 128        │
├─────────────────────────┤
│ D-Pad: N                │
│ Buttons: [X][O][□][△]   │
├─────────────────────────┤
│ Network: Connected      │
│ NRF24: Ready            │
└─────────────────────────┘
```

---

## Task 3: FlashDB Settings Storage 💾

### 3.1 Library Integration
Download FlashDB from: https://gitee.com/Armink/FlashDB

**Directory Structure:**
```
ds-eth-comm/
├── Midware/
│   └── FlashDB/
│       ├── fal/           # Flash Abstraction Layer
│       │   ├── inc/
│       │   └── src/
│       └── flashdb/       # FlashDB core
│           ├── inc/
│           └── src/
```

### 3.2 FAL Configuration
Create `src/flashdb/fal_cfg.h`:

```c
#include "fal.h"

// Flash device table
static struct fal_flash_dev stm32_onchip_flash = {
    "onchip_flash", STM32_FLASH_START_ADRESS, STM32_FLASH_SIZE, (128 * 1024), {NULL, NULL}
};

static struct fal_flash_dev w25q64_flash = {
    "w25q64", W25Q64_FLASH_START_ADRESS, W25Q64_FLASH_SIZE, (4 * 1024), {NULL, NULL}
};

// Flash device table
struct fal_flash_dev *fal_flash_dev_table[] = {
    &stm32_onchip_flash,
    &w25q64_flash,
    NULL
};

// Partition table
static struct fal_partition _partitions[] = {
    {FAL_PART_MAGIC_WORD, "settings", "w25q64", 0, 64 * 1024, 0},
    {FAL_PART_MAGIC_WORD, "data", "w25q64", 64 * 1024, 64 * 1024, 0},
};

// Partition table
struct fal_partition *fal_partition_table[] = {
    &_partitions[0],
    &_partitions[1],
    NULL
};
```

### 3.3 Settings Structure
Create `src/settings/settings.h`:

```c
typedef struct {
    // Network settings
    uint8_t mac_address[6];
    uint32_t ip_address;
    uint32_t gateway;
    uint32_t netmask;
    
    // Controller settings
    uint8_t deadzone;
    uint8_t sensitivity;
    uint8_t button_mapping[16];
    
    // Display settings
    uint8_t brightness;
    uint8_t contrast;
    uint8_t refresh_rate;
    
    // NRF24L01+ settings
    uint8_t channel;
    uint8_t power_level;
    uint8_t data_rate;
    uint64_t pipe_address;
    
    // System settings
    uint8_t debug_level;
    uint8_t auto_save;
    uint32_t crc32;  // For data integrity
} settings_t;
```

### 3.4 KVDB Implementation
Create `src/settings/settings_manager.h` and `src/settings/settings_manager.c`:

**Key Functions:**
- `settings_init()` - Initialize settings system
- `settings_load()` - Load settings from flash
- `settings_save()` - Save settings to flash
- `settings_reset()` - Reset to defaults
- `settings_get_*()` - Get specific setting
- `settings_set_*()` - Set specific setting

### 3.5 Makefile Updates
Add to `Makefile`:

```makefile
# FlashDB source files
CSRC += $(wildcard ./Midware/FlashDB/fal/src/*.c)
CSRC += $(wildcard ./Midware/FlashDB/flashdb/src/*.c)
CSRC += ./src/settings/settings_manager.c

# Include directories
INCDIR += ./Midware/FlashDB/fal/inc
INCDIR += ./Midware/FlashDB/flashdb/inc
INCDIR += ./src/settings
```

---

## Implementation Priority

### Phase 1: Core Infrastructure
1. **FlashDB Integration** - Set up persistent storage
2. **Settings Management** - Implement configuration system

### Phase 2: Display System
3. **OLED Driver** - Basic display functionality
4. **Controller Display** - Show real-time data

### Phase 3: Wireless Communication
5. **NRF24L01+ Driver** - Basic transmission
6. **Data Integration** - Combine with existing UDP system

### Phase 4: System Integration
7. **Settings UI** - Display and modify settings
8. **Error Handling** - Robust error recovery
9. **Testing** - Comprehensive system testing

---

## File Structure After Implementation

```
ds-eth-comm/
├── src/
│   ├── nrf24l01/
│   │   ├── nrf24l01.h
│   │   └── nrf24l01.c
│   ├── oled/
│   │   ├── oled.h
│   │   ├── oled.c
│   │   ├── oled_display.h
│   │   └── oled_display.c
│   ├── settings/
│   │   ├── settings.h
│   │   ├── settings_manager.h
│   │   └── settings_manager.c
│   └── flashdb/
│       └── fal_cfg.h
├── Midware/
│   └── FlashDB/
│       ├── fal/
│       └── flashdb/
└── main.c (updated)
```

---

## Testing Strategy

### Unit Tests
- Test each driver independently
- Verify FlashDB read/write operations
- Test OLED display functions

### Integration Tests
- Controller data flow: HID → Display → Storage
- Wireless transmission reliability
- Settings persistence across power cycles

### System Tests
- Long-term operation stability
- Error recovery scenarios
- Performance under load

---

## Notes

- All new code should follow existing project coding standards
- Use ChibiOS threading and synchronization primitives
- Implement proper error handling and logging
- Consider power consumption for battery operation
- Document all public APIs

---

*This document will be updated as implementation progresses.*
