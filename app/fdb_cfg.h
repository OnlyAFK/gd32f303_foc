/*
 * FlashDB configuration for the GD32F303 FOC firmware.
 *
 * KVDB-only (parameter storage). TSDB is intentionally disabled: there is no
 * RTC, so a time-series log would only carry a boot-relative timestamp that
 * resets on every power cycle. The "last fault" history is kept as a small
 * blob inside KVDB instead (see app/params).
 */
#ifndef _FDB_CFG_H_
#define _FDB_CFG_H_

/* using KVDB feature */
#define FDB_USING_KVDB

/* TSDB disabled — see file header. */
/* #define FDB_USING_TSDB */

/* Using FAL storage mode (on-chip flash via bsp/flash_port.c) */
#define FDB_USING_FAL_MODE

/* Flash write granularity, unit: bit. GD32F30x on-chip flash = 32-bit word
 * program, same as STM32F1. */
#define FDB_WRITE_GRAN                 32

/* Little Endian (default). */
/* #define FDB_BIG_ENDIAN */

/* Route all FlashDB logs to the UART console instead of the C library printf.
 * The project does not retarget printf, so the default would resolve to
 * semihosting and HANG without a debugger. fdb_uart_printf() is implemented in
 * bsp/flash_port.c (vsnprintf -> uart_write). */
void fdb_uart_printf(const char *fmt, ...);
#define FDB_PRINT(...)                 fdb_uart_printf(__VA_ARGS__)

/* Routine info logs (version banner, etc.) stay on; flip to verbose with
 * FDB_DEBUG_ENABLE during bring-up if a mount fails. */
/* #define FDB_DEBUG_ENABLE */

#endif /* _FDB_CFG_H_ */
