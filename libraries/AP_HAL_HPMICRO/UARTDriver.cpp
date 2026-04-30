/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <AP_HAL_HPMICRO/UARTDriver.h>
#include <AP_Math/AP_Math.h>

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

namespace HPMicro
{

UARTDesc uart_desc[] = {HAL_HPM_UART_DEVICES};

static UARTDriver* uart_instances[20] = {nullptr};

UARTDriver::UARTDriver(uint8_t serial_num)
    : AP_HAL::UARTDriver()
{
    _initialized = false;
    uart_num = serial_num;
    uart_instances[uart_desc[uart_num].idx] = this;
}
UARTDriver::UARTDriver(uint8_t serial_num, uint32_t baudrate, bool start)
    : AP_HAL::UARTDriver()
{
    _initialized = false;
    uart_num = serial_num;
    uart_instances[uart_desc[uart_num].idx] = this;
    _begin(baudrate, 0, 0);
}

void UARTDriver::vprintf(const char *fmt, va_list ap)
{
    AP_HAL::UARTDriver::vprintf(fmt, ap);
}

void UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    hpm_stat_t stat;
    uart_config_t config = {0};
    if (b == 0 && txS == 0 && rxS == 0 && _initialized) {
        // the thread owning this port has changed
        _uart_owner_thd = xTaskGetCurrentTaskHandle();
        return;
    }

    if (uart_num < ARRAY_SIZE(uart_desc)) {
        const UARTDesc &desc = uart_desc[uart_num];
        UART_Type *p = desc.port;
        base = p;
        uart_default_config(p, &config);
        if (!_initialized) {
            if (desc.tx == 0) {
                return;
            }
            HPM_IOC->PAD[desc.tx].FUNC_CTL = desc.tx_af;
            HPM_IOC->PAD[desc.rx].FUNC_CTL = desc.rx_af;
            HPM_PIOC->PAD[desc.tx].FUNC_CTL = desc.tx_paf;
            HPM_PIOC->PAD[desc.rx].FUNC_CTL = desc.rx_paf;
            HPM_BIOC->PAD[desc.tx].FUNC_CTL = desc.tx_baf;
            HPM_BIOC->PAD[desc.rx].FUNC_CTL = desc.rx_baf;
            if (desc.cts != 0) {
                HPM_IOC->PAD[desc.cts].FUNC_CTL = desc.cts_af;
                if (desc.cts_paf != 0) {
                    HPM_PIOC->PAD[desc.cts].FUNC_CTL = desc.cts_paf;
                }
                if (desc.cts_baf != 0) {
                    HPM_BIOC->PAD[desc.cts].FUNC_CTL = desc.cts_baf;
                }
            }
            if (desc.rts != 0) {
                HPM_IOC->PAD[desc.rts].FUNC_CTL = desc.rts_af;
                if (desc.rts_paf != 0) {
                    HPM_PIOC->PAD[desc.rts].FUNC_CTL = desc.rts_paf;
                }
                if (desc.rts_baf != 0) {
                    HPM_BIOC->PAD[desc.rts].FUNC_CTL = desc.rts_baf;
                }
            }
            config.modem_config.auto_flow_ctrl_en = (desc.cts != 0) && (desc.rts != 0);
            clock_add_to_group(desc.clk, 0);
#if defined(CONFIG_UART_FIFO_MODE) && (CONFIG_UART_FIFO_MODE == 1)
            config.fifo_enable = true;
#else
            config.fifo_enable = false;
#endif
            config.baudrate = b;
            config.src_freq_in_hz = clock_get_frequency(desc.clk);

            stat = uart_init(p, &config);
            if (stat != status_success) {
                while(1);
            }
            _readbuf.set_size(RX_BUF_SIZE);
            _writebuf.set_size(TX_BUF_SIZE);
            _uart_owner_thd = xTaskGetCurrentTaskHandle();

            uart_enable_irq(p, uart_intr_rx_data_avail_or_timeout);
            intc_m_enable_irq_with_priority(desc.irq_num, 1);
            _initialized = true;
        } else {
            flush();
            config.src_freq_in_hz = clock_get_frequency(desc.clk);
            uart_set_baudrate(p, b, config.src_freq_in_hz);

        }
    }
    _baudrate = b;
}

/**
 * @brief Close UART port
 * @details Disables UART port and clears buffer sizes
 */
void UARTDriver::_end()
{
    if (_initialized) {
        _readbuf.set_size(0);
        _writebuf.set_size(0);
    }
    _initialized = false;
}


void UARTDriver::_flush()
{
    UART_Type *p = uart_desc[uart_num].port;
    _write_mutex.take_blocking();

    do {
        // Peek at bytes in buffer without consuming them
        _buffer_avalable = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        
        if (_buffer_avalable > 0) {
            _buffer_idx = 0;
            
            // NOP for UART0 (likely a timing workaround)
            if (uart_num == 0) {
                __asm volatile ("nop");
            }
            
            // Send first byte
            uart_send_data(p, &_buffer[0], sizeof(_buffer));
            
            // Advance buffer pointer
            _writebuf.advance(_buffer_avalable);
        }
    } while (_buffer_avalable > 0);

    _write_mutex.give();
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}


/**
 * @brief Check available bytes in receive buffer
 * @return Number of bytes available to read
 */
uint32_t UARTDriver::_available()
{
    if (!_initialized) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t UARTDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);

}

ssize_t  UARTDriver::_read(uint8_t *buffer, uint16_t count)
{

    if (!_initialized) {
        return -1;
    }

    const uint32_t ret = _readbuf.read(buffer, count);
    if (ret == 0) {
        return 0;
    }


    _receive_timestamp_update();

    return ret;
}

/**
 * @brief Timer tick handler for UART
 * @details Called periodically to handle UART data operations
 */
void  UARTDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }
    read_data();
    write_data();
}

/**
 * @brief Read data from UART to buffer
 * @details Checks for available data and reads it into the receive buffer
 */
void UARTDriver::read_data()
{
    if (!_initialized) {
        return;
    }
    // Read data from UART to buffer
    while (uart_check_status(base, uart_stat_data_ready)) {
        uint8_t byte;
        byte = uart_read_byte(base);
        _readbuf.write(&byte, 1);
    }
}

/**
 * @brief Write data from buffer to UART
 * @details Sends data from the transmit buffer to the UART hardware
 */
void UARTDriver::write_data()
{
    UART_Type *p = uart_desc[uart_num].port;
    _write_mutex.take_blocking();
    
    // Check if already transmitting
    if (uart_get_enabled_irq(p) & uart_intr_tx_slot_avail) {
        // Already transmitting
        _write_mutex.give();
        return;
    }

    do {
        // Peek at bytes in buffer without consuming them
        _buffer_avalable = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        
        if (_buffer_avalable > 0) {
            _buffer_idx = 0;
            
            // NOP for UART0 (likely a timing workaround)
            if (uart_num == 0) {
                __asm volatile ("nop");
            }
            
            // Send first byte
            
            // If more bytes to send, enable transmit interrupt
            if (_buffer_avalable > 1) {
                uart_enable_irq(p, uart_intr_tx_slot_avail);
                while(status_success != uart_send_byte(p, _buffer[0])) {
                    // Wait for tx fifo empty
                    //TODO, add later version UART ip support
                }
                _write_buffer_output_mutex.wait_blocking();
            } else {
                while(status_success != uart_send_byte(p, _buffer[0])) {
                    // Wait for tx fifo empty
                    //TODO, add later version UART ip support
                }
            }
            
            // Advance buffer pointer
            _writebuf.advance(_buffer_avalable);
        }
    } while (_buffer_avalable > 0);
    
    _write_mutex.give();
}


/**
 * @brief Write data to UART
 * @param buffer Data buffer to write
 * @param size Number of bytes to write
 * @return Number of bytes written
 * @details Writes data directly to UART hardware
 */
size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();

    _writebuf.write(buffer, size);

    _write_mutex.give();
    return size;
}

bool UARTDriver::_discard_input()
{
    if (!_initialized) {
        return false;
    }

    _readbuf.clear();

    return true;
}

// record timestamp of new incoming data
void  UARTDriver::_receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
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
  A return value of zero means the HAL does not support this API
*/
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0) {
        // assume 10 bits per byte. For USB we assume zero transport delay
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

/**
 * @brief UART interrupt service routine
 * @param num UART instance number
 * @details Handles UART interrupts for both receive and transmit operations
 */
void uart_isr(uint8_t num)
{
    UARTDriver *ins = uart_instances[num];
    assert(ins != nullptr);
    
    uint8_t irq_id = uart_get_irq_id(ins->base);
    
    // Handle receive data available interrupt
    if (irq_id == uart_intr_id_rx_data_avail) {
        while (uart_check_status(ins->base, uart_stat_data_ready)) {
            uint8_t byte;
            byte = uart_read_byte(ins->base);
            ins->_readbuf.write(&byte, 1);
        }
    }
    
    // Handle transmit slot available interrupt
    if (irq_id == uart_intr_id_tx_slot_avail) {
        if(ins->_buffer_avalable == 0) {
            uart_disable_irq(ins->base, uart_intr_tx_slot_avail);
            return;
        }
        
        ins->_buffer_idx++;
        if (ins->_buffer_avalable > ins->_buffer_idx) {
            uart_write_byte(ins->base, ins->_buffer[ins->_buffer_idx]);
        } else {
            uart_disable_irq(ins->base, uart_intr_tx_slot_avail);
            ins->_write_buffer_output_mutex.signal_ISR();
        }
    }
}
#include "hpm_soc_irq.h"
#ifdef IRQn_UART0
SDK_DECLARE_EXT_ISR_M(IRQn_UART0, uart_isr0)
void uart_isr0(void)
{
    uart_isr(0);
}
#endif
#ifdef IRQn_UART1
SDK_DECLARE_EXT_ISR_M(IRQn_UART1, uart_isr1)
void uart_isr1(void)
{
    uart_isr(1);
}
#endif
#ifdef IRQn_UART2
SDK_DECLARE_EXT_ISR_M(IRQn_UART2, uart_isr2)
void uart_isr2(void)
{
    uart_isr(2);
}
#endif
#ifdef IRQn_UART3
SDK_DECLARE_EXT_ISR_M(IRQn_UART3, uart_isr3)
void uart_isr3(void)
{
    uart_isr(3);
}
#endif
#ifdef IRQn_UART4
SDK_DECLARE_EXT_ISR_M(IRQn_UART4, uart_isr4)
void uart_isr4(void)
{
    uart_isr(4);
}
#endif
#ifdef IRQn_UART5
SDK_DECLARE_EXT_ISR_M(IRQn_UART5, uart_isr5)
void uart_isr5(void)
{
    uart_isr(5);
}
#endif
#ifdef IRQn_UART6
SDK_DECLARE_EXT_ISR_M(IRQn_UART6, uart_isr6)
void uart_isr6(void)
{
    uart_isr(6);
}
#endif
#ifdef IRQn_UART7
SDK_DECLARE_EXT_ISR_M(IRQn_UART7, uart_isr7)
void uart_isr7(void)
{
    uart_isr(7);
}
#endif
#ifdef IRQn_UART8
SDK_DECLARE_EXT_ISR_M(IRQn_UART8, uart_isr8)
void uart_isr8(void)
{
    uart_isr(8);
}
#endif
#ifdef IRQn_UART9
SDK_DECLARE_EXT_ISR_M(IRQn_UART9, uart_isr9)
void uart_isr9(void)
{
    uart_isr(9);
}
#endif
#ifdef IRQn_UART10
SDK_DECLARE_EXT_ISR_M(IRQn_UART10, uart_isr10)
void uart_isr10(void)
{
    uart_isr(10);
}
#endif
#ifdef IRQn_UART11
SDK_DECLARE_EXT_ISR_M(IRQn_UART11, uart_isr11)
void uart_isr11(void)
{
    uart_isr(11);
}
#endif
#ifdef IRQn_UART12
SDK_DECLARE_EXT_ISR_M(IRQn_UART12, uart_isr12)
void uart_isr12(void)
{
    uart_isr(12);
}
#endif
#ifdef IRQn_UART13
SDK_DECLARE_EXT_ISR_M(IRQn_UART13, uart_isr13)
void uart_isr13(void)
{
    uart_isr(13);
}
#endif
#ifdef IRQn_UART14
SDK_DECLARE_EXT_ISR_M(IRQn_UART14, uart_isr14)
void uart_isr14(void)
{
    uart_isr(14);
}
#endif
#ifdef IRQn_UART15
SDK_DECLARE_EXT_ISR_M(IRQn_UART15, uart_isr15)
void uart_isr15(void)
{
    uart_isr(15);
}
#endif

}
