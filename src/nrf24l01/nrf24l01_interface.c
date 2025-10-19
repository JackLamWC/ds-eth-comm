#include "nrf24l01_interface.h"

#include "ch.h"
#include "hal.h"
#include <string.h>

#define SPI_BUFFER_SIZE 64

static SPIConfig nrf24l01_spi_config = {
    .circular = false,
    .ssline = LINE_SPI2_NRF24_CS,
    .slave = false,
    .data_cb = NULL,
    .error_cb = NULL,
    .cfg1 = SPI_CFG1_MBR_1 | SPI_CFG1_MBR_2 | SPI_CFG1_MBR_0 | SPI_CFG1_DSIZE_8BITS,
    .cfg2 = SPI_CFG2_CPHA | SPI_CFG2_CPOL
};

uint8_t nrf24l01_interface_spi_init(void) {
    msg_t msg;
    msg = spiStart(&SPID2, &nrf24l01_spi_config);
    if (msg != HAL_RET_SUCCESS) {
        return 1;
    }
    return 0;
}

uint8_t nrf24l01_interface_gpio_init(void) {
    return 0;
}

uint8_t nrf24l01_interface_spi_deinit(void) {
    spiStop(&SPID2);
    return 0;
}

uint8_t nrf24l01_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len) {
    static uint8_t spi_tx_buffer[SPI_BUFFER_SIZE];
    static uint8_t spi_rx_buffer[SPI_BUFFER_SIZE];
    
    // Null pointer check
    if (buf == NULL) {
        return 1;  // Error: output buffer is NULL
    }
    
    // Buffer size validation
    if (len == 0 || len > SPI_BUFFER_SIZE) {
        return 1;  // Error: invalid length (0 or exceeds buffer size)
    }
    
    // Prepare SPI transaction: send command, receive data
    memset(spi_tx_buffer, 0, len + 1);
    memset(spi_rx_buffer, 0, len + 1);
    spi_tx_buffer[0] = reg;  // reg already contains the read command + register address
    
    // Cache management for DMA coherency
    SCB_CleanDCache_by_Addr((uint32_t *)spi_tx_buffer, len + 1);  // Ensure DMA sees CPU data
    SCB_InvalidateDCache_by_Addr((uint32_t *)spi_rx_buffer, len + 1);  // Ensure CPU sees DMA data
    
    spiAcquireBus(&SPID2);
    spiSelect(&SPID2);
    msg_t msg = spiExchange(&SPID2, len + 1, spi_tx_buffer, spi_rx_buffer);
    spiUnselect(&SPID2);
    spiReleaseBus(&SPID2);
    
    // Ensure received data is visible to CPU
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)spi_rx_buffer, len + 1);
    
    // Copy received data to user buffer (skip the first byte which is the command response)
    memcpy(buf, spi_rx_buffer + 1, len);
    
    if (msg == HAL_RET_SUCCESS) {
        return 0;
    } else {
        return 1;
    }
}

uint8_t nrf24l01_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len) {
    static uint8_t spi_tx_buffer[SPI_BUFFER_SIZE];
    
    // Buffer size validation - allow NULL buf for command-only operations
    if (len >= SPI_BUFFER_SIZE) {  // Note: reg is already the command+register
        return 1;  // Error: would exceed buffer size
    }
    
    // Copy data to static buffer
    memset(spi_tx_buffer, 0, SPI_BUFFER_SIZE);
    spi_tx_buffer[0] = reg;  // reg already contains the command+register
    
    // Only copy data if buffer is provided and length > 0
    if (buf != NULL && len > 0) {
        memcpy(spi_tx_buffer + 1, buf, len);
    }
    
    // Cache management for DMA coherency
    SCB_CleanDCache_by_Addr((uint32_t *)spi_tx_buffer, len + 1);  // Ensure DMA sees CPU data
    spiAcquireBus(&SPID2);
    spiSelect(&SPID2);
    msg_t msg = spiSend(&SPID2, len + 1, spi_tx_buffer);
    spiUnselect(&SPID2);
    spiReleaseBus(&SPID2);
    
    if (msg == HAL_RET_SUCCESS) {
        return 0;
    } else {
        return 1;
    }
}

uint8_t nrf24l01_interface_gpio_deinit(void) {
    return 0;
}

uint8_t nrf24l01_interface_gpio_write(uint8_t data) {
    if(data) {
        palSetLine(LINE_SPI2_NRF24_CE);
    } else {
        palClearLine(LINE_SPI2_NRF24_CE);
    }
    return 0;
}

void nrf24l01_interface_delay_ms(uint32_t ms) {
    chThdSleepMilliseconds(ms);
}

void nrf24l01_interface_receive_callback(uint8_t type, uint8_t num, uint8_t *buf, uint8_t len) {
    // Null pointer check
    if (buf == NULL) {
        return;  // Early return if buffer is NULL
    }
    
    // Length validation
    if (len == 0) {
        return;  // Early return if length is 0
    }
    
    // TODO: Implement actual callback functionality here
    // This is currently a placeholder function
    return;
}