#include "fdb_port.h"
#include "flashdb.h"
#include "fdb_def.h"
#include "w25qxx.h"
#include "w25qxx_interface.h"
#include "SEGGER_RTT_Channel.h"
#include "ch.h"
#include "chprintf.h"

// from https://blog.csdn.net/u010665511/article/details/151350945
static struct fdb_kvdb kvdb = {0};
static w25qxx_handle_t w25qxx_handle;

// Mutex for thread safety
static mutex_t fdb_mutex;

// Default KV configuration
static struct fdb_default_kv_node default_kv_table[] = {
    {"device_name", "ds-eth-comm", 0},        // 0 means string value
};

static struct fdb_default_kv default_kv = {
    .kvs = default_kv_table,
    .num = sizeof(default_kv_table) / sizeof(default_kv_table[0])
};

// Lock/unlock functions for thread safety
static void fdb_lock(fdb_db_t db) {
    (void)db;  // Unused parameter
    chMtxLock(&fdb_mutex);
}

static void fdb_unlock(fdb_db_t db) {
    (void)db;  // Unused parameter
    chMtxUnlock(&fdb_mutex);
}

static int w25q_init(void) {
  DRIVER_W25QXX_LINK_INIT(&w25qxx_handle, w25qxx_handle_t);
  DRIVER_W25QXX_LINK_SPI_QSPI_INIT(&w25qxx_handle, w25qxx_interface_spi_qspi_init);
  DRIVER_W25QXX_LINK_SPI_QSPI_DEINIT(&w25qxx_handle, w25qxx_interface_spi_qspi_deinit);
  DRIVER_W25QXX_LINK_SPI_QSPI_WRITE_READ(&w25qxx_handle, w25qxx_interface_spi_qspi_write_read);
  DRIVER_W25QXX_LINK_DELAY_MS(&w25qxx_handle, w25qxx_interface_delay_ms);
  DRIVER_W25QXX_LINK_DELAY_US(&w25qxx_handle, w25qxx_interface_delay_us);
  DRIVER_W25QXX_LINK_DEBUG_PRINT(&w25qxx_handle, w25qxx_interface_debug_print);
  w25qxx_set_interface(&w25qxx_handle, W25QXX_INTERFACE_SPI);
  w25qxx_set_type(&w25qxx_handle, W25Q64);
  w25qxx_init(&w25qxx_handle);
  uint8_t manufacturer;
  uint8_t manufacturer_id[2];
  w25qxx_get_jedec_id(&w25qxx_handle, &manufacturer, manufacturer_id);
  chprintf((BaseSequentialStream *)&RTT_S0, "Manufacturer: 0x%02x, Manufacturer ID: 0x%02x%02x\n", manufacturer, manufacturer_id[0], manufacturer_id[1]);
  return 0;
}


static int w25q_read(long addr, uint8_t *buf, size_t size)
{
    chprintf((BaseSequentialStream *)&RTT_S0, "w25q_read: addr: %ld, size: %ld\n", addr, size);
    w25qxx_read(&w25qxx_handle, addr, buf, size);
    return 0;
}


static int w25q_write(long addr, const uint8_t *buf, size_t size)
{
    chprintf((BaseSequentialStream *)&RTT_S0, "w25q_write: addr: %ld, size: %ld\n", addr, size);
    w25qxx_write(&w25qxx_handle, addr, (uint8_t*)buf, size); 
    return 0;
}


static int w25q_erase(long addr, size_t size)
{
    
    for (uint32_t i = 0; i < size; i += 4*1024) {
        w25qxx_sector_erase_4k(&w25qxx_handle, addr + i); 
    }
    return 0;
}


const struct fal_flash_dev w25q64_flash =
{
    .name       = "w25q64",          /* Flash名字（要和fal_cfg.h里的一致） */
    .addr       = 0,                  /* Flash起始地址（外部Flash一般从0开始） */
    .len        = 8 * 1024 * 1024,   /* Flash总大小（W25Q128是16MB） */
    .blk_size   = 4 * 1024,           /* 扇区大小（W25Q是4KB） */
    .ops        = {w25q_init, w25q_read, w25q_write, w25q_erase},  /* 读写擦除函数 */
    .write_gran = FDB_WRITE_GRAN      /* 写入粒度（和fdb_cfg.h里的一致） */
};

int flashdb_init()
{
    fdb_err_t result;  /* FlashDB的错误码 */
    
    /* Initialize mutex for thread safety */
    chMtxObjectInit(&fdb_mutex);

    w25q_init();

    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_LOCK, (void *)fdb_lock);
    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_UNLOCK, (void *)fdb_unlock);
    
    /* 初始化KVDB：参数依次是KVDB对象、Flash名、分区名、默认KV、用户数据 */
    result = fdb_kvdb_init(&kvdb, "w25q64", "kvdb1", &default_kv, NULL);
    
    /* 检查初始化结果 */
    if (result != FDB_NO_ERR) {
        chprintf((BaseSequentialStream *)&RTT_S0, "KVDB init failed! Error code: %d\n", result);
        return -1;  /* 失败返回-1 */
    }

    chprintf((BaseSequentialStream *)&RTT_S0, "KVDB init success! Error code: %d\n", result);  /* 打印成功日志 */
    
    return 0;  /* 成功返回0 */
}

uint8_t flashdb_get_kv_value(const char *key, char *value, size_t value_size)
{
    struct fdb_blob blob;
    fdb_kv_get_blob(&kvdb, key, fdb_blob_make(&blob, value, value_size));
        /* the blob.saved.len is more than 0 when get the value successful */
    if (blob.saved.len > 0) {
        return 0;  // success
    } else {
        return 1;  // failed
    }
}

uint8_t flashdb_set_kv_value(const char *key, const char *value, size_t value_size)
{
    struct fdb_blob blob;
    fdb_kv_set_blob(&kvdb, key, fdb_blob_make(&blob, value, value_size));
    if (blob.saved.len > 0) {
        return 0;  // success
    } else {
        return 1;  // failed
    }
}

uint8_t flashdb_del_kv_value(const char *key)
{
    if (fdb_kv_del(&kvdb, key) == FDB_NO_ERR) {
        return 0;  // success
    } else {
        return 1;  // failed
    }
}