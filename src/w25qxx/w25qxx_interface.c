/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_w25qxx_interface_template.c
 * @brief     driver w25qxx interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2021-07-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/07/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "w25qxx_interface.h"
#include <string.h>
#include "core_cm7.h"  // For cache management functions

// Static buffers for DMA-compatible SPI communication
// DMA requires memory to be in a specific region accessible to the DMA controller
// Buffer size chosen to accommodate w25qxx operations (SFDP, security registers use 256 bytes)
#define SPI_BUFFER_SIZE 512


static const SPIConfig spi_config = {
   .circular = false,
   .ssline = LINE_SPI_FLASH_CS,
   .slave = false,
   .data_cb = NULL,
   .error_cb = NULL,
   .cfg1 = SPI_CFG1_MBR_1 | SPI_CFG1_MBR_2 | SPI_CFG1_DSIZE_8BITS,
   .cfg2 = SPI_CFG2_CPHA | SPI_CFG2_CPOL
 };

 /**
  * @brief  interface spi qspi bus init
  * @return status code
  *         - 0 success
  *         - 1 spi qspi init failed
  * @note   none
  */
 uint8_t w25qxx_interface_spi_qspi_init(void)
 {
    msg_t msg;
    msg = spiStart(&SPID1, &spi_config);
    if (msg != HAL_RET_SUCCESS) {
        return 1;
    }
    return 0;
 }
 
 /**
  * @brief  interface spi qspi bus deinit
  * @return status code
  *         - 0 success
  *         - 1 spi qspi deinit failed
  * @note   none
  */
 uint8_t w25qxx_interface_spi_qspi_deinit(void)
 {
    spiStop(&SPID1);
    return 0;
 }
 
 /**
  * @brief      interface spi qspi bus write read
  * @param[in]  instruction sent instruction
  * @param[in]  instruction_line instruction phy lines
  * @param[in]  address register address
  * @param[in]  address_line address phy lines
  * @param[in]  address_len address length
  * @param[in]  alternate register address
  * @param[in]  alternate_line alternate phy lines
  * @param[in]  alternate_len alternate length
  * @param[in]  dummy dummy cycle
  * @param[in]  *in_buf pointer to a input buffer
  * @param[in]  in_len input length
  * @param[out] *out_buf pointer to a output buffer
  * @param[in]  out_len output length
  * @param[in]  data_line data phy lines
  * @return     status code
  *             - 0 success
  *             - 1 write read failed
  * @note       none
  */
uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
                                             uint32_t address, uint8_t address_line, uint8_t address_len,
                                             uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
                                             uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
                                             uint8_t *out_buf, uint32_t out_len, uint8_t data_line)
{
   static uint8_t spi_tx_buffer[SPI_BUFFER_SIZE];
   static uint8_t spi_rx_buffer[SPI_BUFFER_SIZE];
   msg_t msg;
   uint32_t total_len = in_len + out_len;
   
   // Copy input data
   memcpy(spi_tx_buffer, in_buf, in_len);

   // Cache management for DMA coherency
   SCB_CleanDCache_by_Addr((uint32_t *)spi_tx_buffer, total_len);  // Ensure DMA sees CPU data
   SCB_InvalidateDCache_by_Addr((uint32_t *)spi_rx_buffer, total_len);  // Ensure CPU sees DMA data

   spiSelect(&SPID1);
   
   spiExchange(&SPID1, total_len, spi_tx_buffer, spi_rx_buffer);
   
   // Ensure received data is visible to CPU
   SCB_CleanInvalidateDCache_by_Addr((uint32_t *)spi_rx_buffer, total_len);
   
   memcpy(out_buf, spi_rx_buffer + in_len, out_len);
   spiUnselect(&SPID1);
   return 0;
}
 
 /**
  * @brief     interface delay ms
  * @param[in] ms time
  * @note      none
  */
 void w25qxx_interface_delay_ms(uint32_t ms)
 {
    chThdSleepMilliseconds(ms);
 }
 
 /**
  * @brief     interface delay us
  * @param[in] us time
  * @note      none
  */
 void w25qxx_interface_delay_us(uint32_t us)
 {
    chThdSleepMicroseconds(us);
 }
 
//  /**
//   * @brief     interface print format data
//   * @param[in] fmt format data
//   * @note      none
//   */
//  extern void debug_print(const char *const fmt, ...);
//  void w25qxx_interface_debug_print(const char *const fmt, ...)
//  {
//      va_list args;
//      va_start(args, fmt);
//      debug_print(fmt, args);
//      va_end(args);
//  }