/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_HPM_Namespace.h"
#include <AP_Common/Bitmask.h>
#include <AP_FlashStorage/AP_FlashStorage.h>
#include "hpm_romapi.h"
#include "hpm_common.h"
#include "hpm_l1c_drv.h"
#include "board.h"

#define STORAGE_SIZE HAL_STORAGE_SIZE
#define STORAGE_SECTOR_SIZE (16*1024)

#define STORAGE_LINE_SHIFT 3

#define STORAGE_LINE_SIZE (1<<STORAGE_LINE_SHIFT)
#define STORAGE_NUM_LINES (STORAGE_SIZE/STORAGE_LINE_SIZE)

class HPMicro::Storage : public AP_HAL::Storage
{
public:
    void init() override {}
    void read_block(void *dst, uint16_t src, size_t n) override;
    void write_block(uint16_t dst, const void* src, size_t n) override;

    void _timer_tick(void) override;
    bool healthy(void) override;
    bool get_storage_ptr(void *&ptr, size_t &size) override;

private:
    volatile bool _initialised;
    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[STORAGE_SIZE] __attribute__((aligned(4)));
    Bitmask<STORAGE_NUM_LINES> _dirty_mask;
    xpi_nor_config_t _xpi_nor_config;
    uint32_t _flash_size;
    uint32_t _sector_size;
    uint32_t _page_size;

    bool _flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    bool _flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    bool _flash_erase_sector(uint8_t sector);
    bool _flash_erase_ok(void);
    bool _flash_failed;
    uint32_t _last_re_init_ms;
    uint32_t _last_empty_ms;

    AP_FlashStorage _flash{_buffer,
                        STORAGE_SECTOR_SIZE,
                        FUNCTOR_BIND_MEMBER(&Storage::_flash_write_data, bool, uint8_t, uint32_t, const uint8_t *, uint16_t),
                        FUNCTOR_BIND_MEMBER(&Storage::_flash_read_data, bool, uint8_t, uint32_t, uint8_t *, uint16_t),
                        FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_sector, bool, uint8_t),
                        FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_ok, bool)};

    void _flash_load(void);
    void _flash_write(uint16_t line);
};
