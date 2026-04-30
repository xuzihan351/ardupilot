/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "Scheduler.h"
#include "Semaphores.h"
#include <stdio.h>
#include "hpm_gpio_drv.h"
#include "hpm_i2c_drv.h"
#include "hpm_soc.h"
#include "hpm_ioc_regs.h"
#include "board.h"

using namespace HPMicro;

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();
#define MHZ (1000U*1000U)
#define KHZ (1000U)

// #define I2CDEBUG 1
I2CBusDesc i2c_bus_desc[] = { HAL_HPM_I2C_BUSES };

I2CBus I2CDeviceManager::businfo[ARRAY_SIZE(i2c_bus_desc)];

I2CDeviceManager::I2CDeviceManager(void)
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    for (uint8_t i=0; i<ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].soft) {
            while(1) {
                hpmhal.scheduler->delay(1000);
            }
        } else {
            HPM_IOC->PAD[i2c_bus_desc[i].sda].FUNC_CTL = i2c_bus_desc[i].sda_af | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
            HPM_IOC->PAD[i2c_bus_desc[i].scl].FUNC_CTL = i2c_bus_desc[i].scl_af | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
            HPM_BIOC->PAD[i2c_bus_desc[i].sda].FUNC_CTL = i2c_bus_desc[i].sda_baf;
            HPM_BIOC->PAD[i2c_bus_desc[i].scl].FUNC_CTL = i2c_bus_desc[i].scl_baf;
            HPM_PIOC->PAD[i2c_bus_desc[i].sda].FUNC_CTL = i2c_bus_desc[i].sda_paf;
            HPM_PIOC->PAD[i2c_bus_desc[i].scl].FUNC_CTL = i2c_bus_desc[i].scl_paf;

            HPM_IOC->PAD[i2c_bus_desc[i].sda].PAD_CTL = IOC_PAD_PAD_CTL_OD_MASK;
            HPM_IOC->PAD[i2c_bus_desc[i].scl].PAD_CTL = IOC_PAD_PAD_CTL_OD_MASK;
            i2c_config_t config;
            hpm_stat_t stat;
            uint32_t freq;
            clock_add_to_group(i2c_bus_desc[i].bus_clock, 0);
            freq = clock_get_frequency(i2c_bus_desc[i].bus_clock);
            config.i2c_mode = i2c_mode_normal;
            config.is_10bit_addressing = false;
            stat = i2c_init_master(i2c_bus_desc[i].port, freq, &config);
            if (stat != status_success) {
                while (1) {
                    hpmhal.scheduler->delay(1000);
                }
            }
            businfo[i].port = i2c_bus_desc[i].port;
            businfo[i].bus_clock = i2c_bus_desc[i].bus_clock;
            businfo[i].soft = false;

        }
    }
}

I2CDevice::I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms) :
    bus(I2CDeviceManager::businfo[busnum]),
    _retries(10),
    _address(address),
    _timeout_ms(timeout_ms)
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    set_device_bus(busnum);
    set_device_address(address);
    pname = (char *)malloc(50);
    sprintf(pname, "I2C:%u:%02x",
             (unsigned)busnum, (unsigned)address);
    hpmhal.console->printf("i2c device constructed %s\n", pname);
}

I2CDevice::~I2CDevice()
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    free(pname);
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address,
                             uint32_t bus_clock,
                             bool use_smbus,
                             uint32_t timeout_ms)
{
    if (bus >= ARRAY_SIZE(i2c_bus_desc)) {
        return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(nullptr);
    }
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(NEW_NOTHROW I2CDevice(bus, address, bus_clock, use_smbus, timeout_ms));
    return dev;
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (!bus.semaphore.check_owner()) {
        hpmhal.console->printf("I2C: not owner of 0x%x\n", (unsigned)get_bus_id());
        return false;
    }

    bool result = false;
    if (bus.soft) {
        while(1) {
            hpmhal.scheduler->delay(1000);
        }
        result = true; // TODO: soft I2C handling
    } else {
        hpm_stat_t stat;
        if (recv_len == 0) {
            if (send_len == 0 || send == nullptr) {
                return false;
            }
            /* write-only: no internal address, send is the data buffer */
            stat = i2c_master_address_write(bus.port, _address, (uint8_t*)&send[0], 1, (uint8_t*)&send[1], send_len - 1);
            result = (stat == status_success);
        } else if (recv_len != 0 && send_len != 0) {
            if (send == nullptr || recv == nullptr) {
                return false;
            }
            /* address read: send contains the internal address bytes */
            stat = i2c_master_address_read(bus.port, _address, (uint8_t*)send, send_len, (uint8_t*)recv, recv_len);
            result = (stat == status_success);
        } else {
            /* other cases are errors */
            return false;
        }
    }

    return result;
}

/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return bus.register_periodic_callback(period_usec, cb, this);
}


/*
  adjust a periodic callback
*/
bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return bus.adjust_timer(h, period_usec);
}

/*
  get mask of bus numbers for all configured I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask(void) const
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return ((1U << ARRAY_SIZE(i2c_bus_desc)) - 1);
}

/*
  get mask of bus numbers for all configured internal I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    uint32_t result = 0;
    for (size_t i = 0; i < ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].internal) {
            result |= (1u << i);
        }
    }
    return result;
}

/*
  get mask of bus numbers for all configured external I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return get_bus_mask() & ~get_bus_mask_internal();
}
