/*
 * FAL on-chip flash port for the GD32F303CC (256 KB flash, 2 KB pages).
 *
 * Backs FlashDB's KVDB partition with the top 8 KB of program flash
 * (0x0803_E000 .. 0x0804_0000). The Keil IROM1 size is reduced to 0x3E000 so
 * the linker never places code into this region (see f303_foc.uvprojx).
 *
 * Adapted from FlashDB's fal_flash_stm32f1_port.c: same 32-bit write
 * granularity, HAL_FLASH_* replaced by the GD32 SPL FMC API.
 *
 * Layering: bsp may use the vendor SPL (libraries) but not foc/. Long erase /
 * program loops reload the free watchdog directly via the SPL
 * (fwdgt_counter_reload) — harmless before the watchdog is started, and avoids
 * an upward dependency on foc/fault.
 */
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <fal.h>
#include "gd32f30x.h"
#include "uart.h"

#define GD_FLASH_PAGE_SIZE   2048U
#define KVDB_FLASH_ADDR      0x0803E000UL   /* top 8 KB of 256 KB flash */
#define KVDB_FLASH_LEN       (8U * 1024U)

/*
 * Log sink for FlashDB + FAL (see FDB_PRINT in fdb_cfg.h / FAL_PRINTF in
 * fal_cfg.h). vsnprintf into a stack buffer then push to the UART console —
 * never touches semihosting, so it is safe with no debugger attached. Called
 * only from the (slow) init / save paths, never from the control ISR.
 */
void fdb_uart_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        uart_write(buf, (uint32_t)((n < (int)sizeof(buf)) ? n : (int)sizeof(buf) - 1));
    }
}

/* GD32 fmc_flag_clear() decodes a single encoded flag (reg index + bit pos)
 * per call — flags must NOT be OR'd together. Clear all three bank0 status
 * bits individually before/after each operation. */
static void gd_clear_flags(void)
{
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
}

static int gd_init(void)
{
    return 1;
}

static int gd_read(long offset, uint8_t *buf, size_t size)
{
    uint32_t addr = gd32_onchip_flash.addr + offset;
    size_t i;

    for (i = 0; i < size; i++) {
        buf[i] = *(volatile uint8_t *)(addr + i);
    }
    return (int)size;
}

static int gd_write(long offset, const uint8_t *buf, size_t size)
{
    uint32_t addr = gd32_onchip_flash.addr + offset;
    uint32_t word;
    size_t i;

    fmc_unlock();
    gd_clear_flags();

    for (i = 0; i < size; i += 4, addr += 4) {
        /* copy into an aligned word so unaligned src buffers are safe */
        memcpy(&word, buf + i, 4);
        if (fmc_word_program(addr, word) != FMC_READY) {
            fmc_lock();
            return -1;
        }
        gd_clear_flags();
        if (*(volatile uint32_t *)addr != word) {   /* read-back verify */
            fmc_lock();
            return -1;
        }
        fwdgt_counter_reload();                      /* flash ops are slow; feed the dog */
    }

    fmc_lock();
    return (int)size;
}

static int gd_erase(long offset, size_t size)
{
    uint32_t addr = gd32_onchip_flash.addr + offset;
    size_t pages = (size + GD_FLASH_PAGE_SIZE - 1) / GD_FLASH_PAGE_SIZE;
    size_t i;

    fmc_unlock();
    gd_clear_flags();

    for (i = 0; i < pages; i++) {
        if (fmc_page_erase(addr + i * GD_FLASH_PAGE_SIZE) != FMC_READY) {
            fmc_lock();
            return -1;
        }
        gd_clear_flags();
        fwdgt_counter_reload();
    }

    fmc_lock();
    return (int)size;
}

/*
 * FAL flash device. `.addr` is the KVDB base, so partition offsets in
 * fal_cfg.h start at 0. `.write_gran` is in bits (32 = word program).
 */
const struct fal_flash_dev gd32_onchip_flash =
{
    .name       = "gd32_onchip",
    .addr       = KVDB_FLASH_ADDR,
    .len        = KVDB_FLASH_LEN,
    .blk_size   = GD_FLASH_PAGE_SIZE,
    .ops        = { gd_init, gd_read, gd_write, gd_erase },
    .write_gran = 32,
};
