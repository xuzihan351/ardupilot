/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_HPMICRO/AP_HAL_HPM.h>
#include <AP_HAL_HPMICRO/Semaphores.h>

#include "FreeRTOS.h"
#include "task.h"
#include "hpm_soc.h"
#include "hpm_uart_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_ioc_regs.h"
namespace HPMicro
{

struct UARTDesc {
    UART_Type *port;
    clock_name_t clk;
    int rx;
    int tx;
    int rx_af;
    int tx_af;
    int rx_baf;
    int tx_baf;
    int rx_paf;
    int tx_paf;
    uint32_t irq_num;
    uint8_t idx;
    int cts;
    int rts;
    int cts_af;
    int rts_af;
    int cts_baf;
    int rts_baf;
    int cts_paf;
    int rts_paf;
};

class UARTDriver : public AP_HAL::UARTDriver
{
public:

    UARTDriver(uint8_t serial_num);

    UARTDriver(uint8_t serial_num, uint32_t baudrate, bool start);

    virtual ~UARTDriver() = default;

    void vprintf(const char *fmt, va_list ap) override;

    bool is_initialized() override;
    bool tx_pending() override;

    uint32_t txspace() override;

    void _timer_tick(void) override;

    uint32_t bw_in_bytes_per_second() const override
    {
        return 10*1024;
    }

    /*
      return timestamp estimate in microseconds for when the start of
      a nbytes packet arrived on the uart. This should be treated as a
      time constraint, not an exact time. It is guaranteed that the
      packet did not start being received after this time, but it
      could have been in a system buffer before the returned time.
      This takes account of the baudrate of the link. For transports
      that have no baudrate (such as USB) the time estimate may be
      less accurate.
      A return value of zero means the HAL does not support this API */
     
    uint64_t receive_time_constraint_us(uint16_t nbytes) override; 

    uint32_t get_baud_rate() const override { return _baudrate; }

    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    BinarySemaphore _write_buffer_output_mutex;
    UART_Type* base;
    uint8_t _buffer[32];
    uint8_t _buffer_avalable;
    uint8_t _buffer_idx;
private:
    bool _initialized;
    const size_t TX_BUF_SIZE = 2048;
    const size_t RX_BUF_SIZE = 2048;
    Semaphore _write_mutex;
    void read_data();
    void write_data();

    uint8_t uart_num;

    // timestamp for receiving data on the UART, avoiding a lock
    uint64_t _receive_timestamp[2];
    uint8_t _receive_timestamp_idx;
    uint32_t _baudrate;
    const tskTaskControlBlock* _uart_owner_thd;

    void _receive_timestamp_update(void);

protected:
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    bool _discard_input() override; // discard all bytes available for reading
};

}
