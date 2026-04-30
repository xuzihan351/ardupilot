/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <AP_HAL/AP_HAL.h>
#include "HAL_HPM_Class.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "Storage.h"
#include "FreeRTOS.h"

// Flash storage base addresses - defined in hwdef.dat
#ifndef HAL_HPM_STORAGE_FLASH_BASE_ADDR
#define HAL_HPM_STORAGE_FLASH_BASE_ADDR 0x80300000
#endif

#ifndef HAL_HPM_STORAGE_OFFSET_ADDR
#define HAL_HPM_STORAGE_OFFSET_ADDR 0x0300000
#endif

// #define STORAGEDEBUG 1

using namespace HPMicro;

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }
    memset((void *)&_xpi_nor_config, 0, sizeof(_xpi_nor_config));
    _dirty_mask.clearall();
    xpi_nor_config_option_t option;
    option.header.U = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    option.option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    option.option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;

    XPI_Type *base = BOARD_APP_XPI_NOR_XPI_BASE;

    hpm_stat_t status = rom_xpi_nor_auto_config(base, &_xpi_nor_config, &option);
    (void)status;
    rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE, &_xpi_nor_config, xpi_nor_property_total_size, &_flash_size);
    rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE, &_xpi_nor_config, xpi_nor_property_sector_size, &_sector_size);
    rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE, &_xpi_nor_config, xpi_nor_property_page_size, &_page_size);
    _flash_load();
    _initialised = true;
}

/*
  mark some lines as dirty. Note that there is no attempt to avoid
  the race condition between this code and the _timer_tick() code
  below, which both update _dirty_mask. If we lose the race then the
  result is that a line is written more than once, but it won't result
  in a line not being written.
*/
void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
#ifdef STORAGEDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    uint16_t end = loc + length;
    for (uint16_t line=loc>>STORAGE_LINE_SHIFT;
         line <= end>>STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
#ifdef STORAGEDEBUG
        hpmhal.console->printf("%s:%d read_block failed \n", __PRETTY_FUNCTION__, __LINE__);
#endif
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
#ifdef STORAGEDEBUG
        hpmhal.console->printf("%s:%d write_block failed \n", __PRETTY_FUNCTION__, __LINE__);
#endif
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    if (!_initialised) {
        return;
    }
    if (_dirty_mask.empty()) {
        _last_empty_ms = AP_HAL::millis();
        return;
    }

    // write out the first dirty line. We don't write more
    // than one to keep the latency of this call to a minimum
    uint16_t i;
    for (i=0; i<STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == STORAGE_NUM_LINES) {
        // this shouldn't be possible
        return;
    }

    // save to storage backend
    _flash_write(i);
}

/*
  load all data from flash
 */
void Storage::_flash_load(void)
{
#ifdef STORAGEDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (!_flash.init()) {
        AP_HAL::panic("unable to init flash storage");
    }
}

/*
  write one storage line. This also updates _dirty_mask.
*/
void Storage::_flash_write(uint16_t line)
{
#ifdef STORAGEDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_flash.write(line*STORAGE_LINE_SIZE, STORAGE_LINE_SIZE)) {
        // mark the line clean
        _dirty_mask.clear(line);
    }
}

/*
  callback to write data to flash
 */
bool Storage::_flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
#ifdef STORAGEDEBUG
    hpmhal.console->printf("%s:%d  \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    hpm_stat_t status;
    uint32_t *src = (uint32_t *)malloc(length + 4);
    uint32_t *src_aligned = (uint32_t *)(((uint32_t)src + 3) & 0xFFFFFFFC);
    if (src == nullptr) {
        return false;
    }
    memcpy(src_aligned, data, length);
    uint32_t address = sector * STORAGE_SECTOR_SIZE + offset + HAL_HPM_STORAGE_OFFSET_ADDR;
    l1c_dc_flush_all();
    portENTER_CRITICAL();
    status = rom_xpi_nor_program(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &_xpi_nor_config, src_aligned, address, (uint32_t)length);
    __asm volatile ("fence.i");
    portEXIT_CRITICAL();
    if (status != status_success) {
        hpmhal.console->printf("Storage: failed at %u:%u for %u - re-init %u\n",
                (unsigned)sector, (unsigned)offset, (unsigned)length, (unsigned)status);
        free(src);
        return false;
    }
    free(src);
    return true;
}

/*
  callback to read data from flash
 */
bool Storage::_flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    uint32_t address = sector * STORAGE_SECTOR_SIZE + offset;
#ifdef STORAGEDEBUG
    hpmhal.console->printf("%s:%d  -> sec:%u off:%u len:%u addr:%x\n", __PRETTY_FUNCTION__, __LINE__,sector,(unsigned int)offset,length,(unsigned int)address);
#endif
    const uint8_t *p = (const uint8_t *)(HAL_HPM_STORAGE_FLASH_BASE_ADDR + address);
    const uint8_t *pend = p + length;
    uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(p);
    uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(pend);
    l1c_dc_invalidate((uint32_t)aligned_start, aligned_end - aligned_start);
    memcpy(data, p, length);
    return true;
}

/*
  callback to erase flash sector
 */
bool Storage::_flash_erase_sector(uint8_t sector)
{
#ifdef STORAGEDEBUG
    hpmhal.console->printf("%s:%d  -> sec:%u\n", __PRETTY_FUNCTION__, __LINE__,sector);
#endif
    uint32_t address = sector * STORAGE_SECTOR_SIZE + HAL_HPM_STORAGE_OFFSET_ADDR;
    hpm_stat_t status;
    portENTER_CRITICAL();
    status = rom_xpi_nor_erase(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &_xpi_nor_config, address, STORAGE_SECTOR_SIZE);
    __asm volatile ("fence.i");
    portEXIT_CRITICAL();
    if (status != status_success) {
        hpmhal.console->printf("Storage: erase failed at sector %u - re-init %u\n",
                (unsigned)sector, (unsigned)status);
        return false;
    }
    return true;
}

/*
  callback to check if erase is allowed
 */
bool Storage::_flash_erase_ok(void)
{
#ifdef STORAGEDEBUG
    hpmhal.console->printf("%s:%d  \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    // only allow erase while disarmed
    return !hpmhal.util->get_soft_armed();
}

/*
  consider storage healthy if we have nothing to write sometime in the
  last 2 seconds
 */
bool Storage::healthy(void)
{
#ifdef STORAGEDEBUG
    hpmhal.console->printf("%s:%d  \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return _initialised && AP_HAL::millis() - _last_empty_ms < 2000;
}

/*
  get storage size and ptr
 */
bool Storage::get_storage_ptr(void *&ptr, size_t &size)
{
    if (!_initialised) {
        return false;
    }
    ptr = _buffer;
    size = sizeof(_buffer);
    return true;
}
