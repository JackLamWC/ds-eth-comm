#ifndef __FDB_PORT_H
#define __FDB_PORT_H

#include "fdb_def.h"

int flashdb_init(void);
uint8_t flashdb_get_kv_value(const char *key, char *value, size_t value_size);
uint8_t flashdb_set_kv_value(const char *key, const char *value, size_t value_size);
uint8_t flashdb_del_kv_value(const char *key);
#endif // __FDB_PORT_H