/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <AP_HAL_HPMICRO/USBVCPDriver.h>
#include <AP_Math/AP_Math.h>
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "usb_osal.h"
#include "semphr.h"
#include "assert.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "usb_osal.h"

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[2][512];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[2048];
volatile uint32_t write_buffer_wr = 0;
volatile uint32_t max_tx = 0, max_rx = 0;
volatile uint8_t read_buffer_index;
volatile bool dtr_enable;
/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x01
#define CDC_INT_EP 0x83
namespace HPMicro
{
USBVCPDriver *USBVCPDriver::_singleton = nullptr;
static void cdc_acm_init(uint8_t busid, uint32_t reg_base);
USBVCPDriver::USBVCPDriver(uint8_t serial_num)
    : AP_HAL::UARTDriver()
{
    _singleton = this;
    _initialized = false;
    _serial_num = serial_num;
    _transmitting = false;
    _begin(0, 0, 0);
}

void USBVCPDriver::vprintf(const char *fmt, va_list ap)
{
    AP_HAL::UARTDriver::vprintf(fmt, ap);
}

void USBVCPDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    (void)b;
    (void)rxS;
    (void)txS;
    if (_serial_num == 0) {
        board_init_usb((USB_Type *)HPM_USB0);
        intc_set_irq_priority(IRQn_USB0, 2);
        cdc_acm_init(0, (uint32_t)HPM_USB0);
    } else if (_serial_num == 1) {
        board_init_usb((USB_Type *)HPM_USB1);
        intc_set_irq_priority(IRQn_USB1, 2);
        cdc_acm_init(1, (uint32_t)HPM_USB1);
    } else {
        while (1) {
            hpmhal.scheduler->delay(1);
        }
    }
    _readbuf.set_size(RX_BUF_SIZE);
    _writebuf.set_size(TX_BUF_SIZE);
    _initialized = true;
}

/**
 * @brief Close USB VCP port
 * @details Disables USB VCP port and clears buffer sizes
 */
void USBVCPDriver::_end()
{
    if (_initialized) {
        _readbuf.set_size(0);
        _writebuf.set_size(0);
    }
    _initialized = false;
}

void USBVCPDriver::_flush()
{
    // Peek at bytes in buffer without consuming them
    _buffer_avalable = _writebuf.peekbytes(_buffer, sizeof(_buffer));
    
    if (_buffer_avalable > 0) {
        
        memcpy(&write_buffer[0], _buffer, _buffer_avalable);

        usbd_ep_start_write(0, CDC_IN_EP, &write_buffer[0], _buffer_avalable);
        _transmitting = true;
    }
    while(_transmitting == true);
}

bool USBVCPDriver::is_initialized()
{
    return _initialized;
}

bool USBVCPDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}


/**
 * @brief Check available bytes in receive buffer
 * @return Number of bytes available to read
 */
uint32_t USBVCPDriver::_available()
{
    if (!_initialized) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t USBVCPDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);
}

ssize_t  USBVCPDriver::_read(uint8_t *buffer, uint16_t count)
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
 * @brief Timer tick handler for USB VCP
 * @details Called periodically to handle USB VCP data operations
 */
void  USBVCPDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }
    // USB VCP data handling will go here
    // For now, we'll just leave this as a placeholder
    read_data();
    write_data();
}

void USBVCPDriver::read_data()
{
}

/**
 * @brief Write data from buffer to UART
 * @details Sends data from the transmit buffer to the UART hardware
 */
void USBVCPDriver::write_data()
{
    if (_transmitting) {
        return;
    }

    // Peek at bytes in buffer without consuming them
    _buffer_avalable = _writebuf.peekbytes(_buffer, sizeof(_buffer));
    
    if (_buffer_avalable > 0) {
        
        memcpy(&write_buffer[0], _buffer, _buffer_avalable);

        usbd_ep_start_write(0, CDC_IN_EP, &write_buffer[0], _buffer_avalable);
        _transmitting = true;
    }
}

/**
 * @brief Write data to USB VCP
 * @param buffer Data buffer to write
 * @param size Number of bytes to write
 * @return Number of bytes written
 * @details Writes data to USB VCP
 */
size_t USBVCPDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();

    // Write data to USB VCP buffer
    size_t ret = _writebuf.write(buffer, size);

    _write_mutex.give();
    return ret;
}

bool USBVCPDriver::_discard_input()
{
    if (!_initialized) {
        return false;
    }

    _readbuf.clear();

    return true;
}

// record timestamp of new incoming data
void  USBVCPDriver::_receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
}

/*
  return timestamp estimate in microseconds for when the start of
  a nbytes packet arrived on the USB VCP. This should be treated as a
  time constraint, not an exact time. It is guaranteed that the
  packet did not start being received after this time, but it
  could have been in a system buffer before the returned time.
  This takes account of the baudrate of the link. For transports
  that have no baudrate (such as USB) the time estimate may be
  less accurate.
  A return value of zero means the HAL does not support this API
*/
uint64_t USBVCPDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0) {
        // For USB we assume zero transport delay since baud rate is not applicable
        // but we'll keep the same interface for compatibility
        uint32_t transport_time_us = 0;
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

// USB VCP interrupt service routine will go here if needed
// For now, we don't need any interrupt handlers

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t config_descriptor_hs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const uint8_t config_descriptor_fs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, 0x01),
};

static const uint8_t other_speed_config_descriptor_hs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t other_speed_config_descriptor_fs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 }, /* Langid */
    "HPMicro",                    /* Manufacturer */
    "HPMicro CDC DEMO",           /* Product */
    "2024051703",                 /* Serial Number */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return config_descriptor_hs;
    } else if (speed == USB_SPEED_FULL) {
        return config_descriptor_fs;
    } else {
        return NULL;
    }
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return device_quality_descriptor;
}

static const uint8_t *other_speed_config_descriptor_callback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return other_speed_config_descriptor_hs;
    } else if (speed == USB_SPEED_FULL) {
        return other_speed_config_descriptor_fs;
    } else {
        return NULL;
    }
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor cdc_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .other_speed_descriptor_callback = other_speed_config_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
    case USBD_EVENT_RESET:
        break;
    case USBD_EVENT_CONNECTED:
        break;
    case USBD_EVENT_DISCONNECTED:
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
        /* setup first out ep read transfer */
        read_buffer_index = 0;
        usbd_ep_start_read(busid, CDC_OUT_EP, &read_buffer[0][0], usbd_get_ep_mps(busid, CDC_OUT_EP));
        break;
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;

    default:
        break;
    }
}

void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    uint8_t index = read_buffer_index;

    read_buffer_index = (index == 0) ? 1 : 0;
    USBVCPDriver *instance = USBVCPDriver::get_singleton();
    instance->_readbuf.write(&read_buffer[index][0], nbytes);
    usbd_ep_start_read(busid, ep, &read_buffer[read_buffer_index][0], usbd_get_ep_mps(busid, ep));
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{

    USBVCPDriver *instance = USBVCPDriver::get_singleton();
    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(busid, ep, NULL, 0);
    } else {
        instance->_writebuf.advance(instance->_buffer_avalable);
        
        instance->_buffer_avalable = instance->_writebuf.peekbytes(instance->_buffer, sizeof(instance->_buffer));
        if (instance->_buffer_avalable > 0) {
            memcpy(&write_buffer[0], instance->_buffer, instance->_buffer_avalable);
            usbd_ep_start_write(busid, ep, &write_buffer[0], instance->_buffer_avalable);
        } else {
            instance->_transmitting = false;
        }
    }
}

/*!< endpoint call back */
struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

/* function ------------------------------------------------------------------*/
struct usbd_interface intf0;
struct usbd_interface intf1;

void cdc_acm_init(uint8_t busid, uint32_t reg_base)
{
    usbd_desc_register(busid, &cdc_descriptor);
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
    usbd_add_endpoint(busid, &cdc_out_ep);
    usbd_add_endpoint(busid, &cdc_in_ep);

    usbd_initialize(busid, reg_base, usbd_event_handler);
}
}
