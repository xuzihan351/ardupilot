/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "HAL_HPM_Class.h"
#include "Scheduler.h"
#include "SPIDevice.h"
#include "I2CDevice.h"
#include "UARTDriver.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "GPIO.h"
#include "Storage.h"
#include "AnalogIn.h"
#include "Util.h"
#ifdef HAL_WITH_USB_VCP
#include "USBVCPDriver.h"
#endif
#if AP_SIM_ENABLED
#include <AP_HAL/SIMState.h>
#endif
static Empty::UARTDriver serial1Driver;
static HPMicro::UARTDriver serial2Driver(2);
static Empty::UARTDriver serial3Driver;
static Empty::UARTDriver serial4Driver;
static Empty::UARTDriver serial5Driver;
static Empty::UARTDriver serial6Driver;
static Empty::UARTDriver serial7Driver;
static Empty::UARTDriver serial8Driver;
static Empty::UARTDriver serial9Driver;
#ifdef HAL_WITH_USB_VCP
#ifndef HAL_USB_PORT_IDX
#define HAL_USB_PORT_IDX 0
#endif
#ifdef HAL_USB_MSP
static HPMicro::USBVCPDriver cons(HAL_USB_PORT_IDX);  //USB VCP as msp port
static HPMicro::UARTDriver serialConsoleDriver(0, 115200, true);  //UART as console port
#else
static HPMicro::UARTDriver cons(1);   //UART as msp port
static HPMicro::USBVCPDriver serialConsoleDriver(HAL_USB_PORT_IDX);  //USB VCP as console port
#endif
#else
static HPMicro::UARTDriver serialConsoleDriver(0, 115200, true); //UART0 as console port
static HPMicro::UARTDriver cons(1);   //UART1 as msp port
#endif
#if HAL_WITH_DSP
static Empty::DSP dspDriver;
#endif

static HPMicro::I2CDeviceManager i2cDeviceManager;
static HPMicro::SPIDeviceManager spiDeviceManager;
#ifndef HAL_DISABLE_ADC_DRIVER
static HPMicro::AnalogIn analogIn;
#else
static Empty::AnalogIn analogIn;
#endif
#ifdef HAL_USE_EMPTY_STORAGE
static Empty::Storage storageDriver;
#else
static HPMicro::Storage storageDriver;
#endif
static HPMicro::GPIO gpioDriver;
#if AP_SIM_ENABLED
static Empty::RCOutput rcoutDriver;
#else
static HPMicro::RCOutput rcoutDriver;
#endif
static HPMicro::RCInput rcinDriver;
static HPMicro::Scheduler schedulerInstance;
static HPMicro::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;
static Empty::Flash flashDriver;

#if AP_SIM_ENABLED
static AP_HAL::SIMState xsimstate;
#endif

ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(8) static uint8_t noncache_memory_pool[1024*24];

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

extern "C" {
    void __wrap_printf(const char *fmt, ...)
    {
    va_list arg;
    va_start(arg, fmt);
    hpmhal.console->printf(fmt, arg);
    va_end(arg);
    }
}

static HPMicro::CANIface* canDrivers[HAL_NUM_CAN_IFACES];

HAL_HPM::HAL_HPM() :
    AP_HAL::HAL(
        &cons, //Console/mavlink
        &serial1Driver, //Telem 1
        &serial2Driver, //Telem 2
        &serial3Driver, //GPS 1
        &serial4Driver, //GPS 2
        &serial5Driver, //Extra 1
        &serial6Driver, //Extra 2
        &serial7Driver, //Extra 3
        &serial8Driver, //Extra 4
        &serial9Driver, //Extra 5
        &i2cDeviceManager,
        &spiDeviceManager,
        nullptr,
        &analogIn,
        &storageDriver,
        &serialConsoleDriver, //Console/mavlink
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
#if AP_SIM_ENABLED
        &xsimstate,
#endif
#if HAL_WITH_DSP
        &dspDriver,
#endif
        (AP_HAL::CANIface**)canDrivers
    )
{
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        canDrivers[i] = nullptr;
    }
}

void HAL_HPM::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    ((HPMicro::Scheduler *)scheduler)->set_callbacks(callbacks);
    utilInstance.malloc_type_init(noncache_memory_pool, sizeof(noncache_memory_pool));
    scheduler->init();
}

void AP_HAL::init()
{
}

