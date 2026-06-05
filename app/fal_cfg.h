/*
 * FAL (Flash Abstraction Layer) configuration for the GD32F303 FOC firmware.
 *
 * One on-chip flash device whose base address is already the parameter region
 * (top 8 KB of the 256 KB flash, see bsp/flash_port.c), so the single KVDB
 * partition starts at offset 0.
 */
#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

/* Route FAL logs to the UART console (same reason as FDB_PRINT in fdb_cfg.h:
 * the project does not retarget printf). fal.h includes this file *before*
 * fal_def.h applies its `#ifndef FAL_PRINTF` default, so this override wins.
 * Do NOT include fal_def.h here: FAL_PART_MAGIC_WORD is only needed where
 * FAL_PART_TABLE is expanded (in fal.c, after fal_def.h is already included). */
void fdb_uart_printf(const char *fmt, ...);
#define FAL_PRINTF                     fdb_uart_printf

#define FAL_PART_HAS_TABLE_CFG

/* ===================== Flash device Configuration ========================= */
extern const struct fal_flash_dev gd32_onchip_flash;

/* flash device table */
#define FAL_FLASH_DEV_TABLE                                          \
{                                                                    \
    &gd32_onchip_flash,                                              \
}

/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
/* partition table: {magic, partition name, flash device name, offset, size, reserved} */
#define FAL_PART_TABLE                                                                 \
{                                                                                      \
    {FAL_PART_MAGIC_WORD, "fdb_kvdb1", "gd32_onchip", 0, 8 * 1024, 0},                 \
}
#endif /* FAL_PART_HAS_TABLE_CFG */

#endif /* _FAL_CFG_H_ */
