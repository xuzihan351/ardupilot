/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "SPIDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "Scheduler.h"
#include "Semaphores.h"
#include <stdio.h>
#include "hpm_gpio_drv.h"
#include "hpm_spi.h"
#include "hpm_soc.h"
#include "hpm_ioc_regs.h"
#include "hpm_l1c_drv.h"
#include "hpm_dmamux_drv.h"

// SPI Configuration Parameters - defined in hwdef.dat
#ifndef HPM_SPI_TIMEOUT_VALUE
#define HPM_SPI_TIMEOUT_VALUE 0xFFFFFFFF  // Default infinite timeout for SPI operations
#endif

using namespace HPMicro;

#define MHZ (1000U*1000U)
#define KHZ (1000U)

// #define SPIDEBUG 1
dmaChannelDescriptor_t dmaDescriptors_hdma[8] = {
    0,

};
dmaChannelDescriptor_t dmaDescriptors_xdma[8] = {
    0,
};

SDK_DECLARE_EXT_ISR_M(IRQn_HDMA, hdma_isr)
void hdma_isr(void)
{
    uint32_t int_stat = HPM_HDMA->INTSTATUS;
    HPM_HDMA->INTSTATUS = int_stat;
    for (int i = 0; i <= 8; i++) {
        if (int_stat & (1 << (DMA_STATUS_TC_SHIFT + i))) {
            AP_HAL::BinarySemaphore *sem = dmaDescriptors_hdma[i].bin_sem;
            if (sem) {
                sem->signal_ISR();
            }
        }
    }
}

SDK_DECLARE_EXT_ISR_M(IRQn_XDMA, xdma_isr)
void xdma_isr(void)
{
    uint32_t int_stat = HPM_XDMA->INTSTATUS;
    HPM_XDMA->INTSTATUS = int_stat;
    for (int i = 0; i <= 8; i++) {
        if (int_stat & (1 << (DMA_STATUS_TC_SHIFT + i))) {
            AP_HAL::BinarySemaphore *sem = dmaDescriptors_xdma[i].bin_sem;
            if (sem) {
                sem->signal_ISR();
            }
        }
    }
}

hpm_stat_t dma_setup_handshake_fixed(DMA_Type *ptr,  dma_handshake_config_fixed_t *pconfig, bool start_transfer)
{
    hpm_stat_t stat = status_success;
    dma_channel_config_t config = {0};
    dma_default_channel_config(ptr, &config);

    if (true == pconfig->dst_fixed) {
        config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
        config.dst_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    }
    if (true == pconfig->src_fixed) {
        config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
        config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    }

    if (pconfig->ch_index >= DMA_SOC_CHANNEL_NUM) {
        return status_invalid_argument;
    }

    config.src_width = pconfig->data_width;
    config.dst_width = pconfig->data_width;
    config.src_addr = pconfig->src;
    config.dst_addr = pconfig->dst;
    config.size_in_byte = pconfig->size_in_byte;
    config.interrupt_mask = pconfig->interrupt_mask;
    /*  In DMA handshake case, source burst size must be 1 transfer, that is 0. */
    config.src_burst_size = 0;
    stat = dma_setup_channel(ptr, pconfig->ch_index, &config, start_transfer);
    if (stat != status_success) {
        return stat;
    }
    return stat;
}
hpm_stat_t spi_tx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, SPI_Type *spi_ptr, uint32_t src, uint8_t data_width, uint32_t size)
{
    dma_handshake_config_fixed_t config = { 0 };

    config.ch_index = ch_num;
    config.dst = (uint32_t)&spi_ptr->DATA;
    config.dst_fixed = true;
    config.src = src;
    config.src_fixed = false;
    config.data_width = data_width;
    config.size_in_byte = size;
    config.interrupt_mask = DMA_INTERRUPT_MASK_ALL;

    return dma_setup_handshake_fixed(dma_ptr, &config, true);
}

hpm_stat_t spi_rx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, SPI_Type *spi_ptr, uint32_t dst, uint8_t data_width, uint32_t size)
{
    dma_handshake_config_t config = {0};

    dma_default_handshake_config(dma_ptr, &config);
    config.ch_index = ch_num;
    config.dst = dst;
    config.dst_fixed = false;
    config.src = (uint32_t)&spi_ptr->DATA;
    config.src_fixed = true;
    config.data_width = data_width;
    config.size_in_byte = size;

    return dma_setup_handshake(dma_ptr, &config, true);
}
static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();
SPIDeviceDesc device_desc[] = {HAL_HPM_SPI_DEVICES};
SPIBusDesc bus_desc[] = {HAL_HPM_SPI_BUSES};

SPIBus::SPIBus(uint8_t _bus):
    DeviceBus(Scheduler::SPI_PRIORITY), bus(_bus)
{
    HPM_IOC->PAD[bus_desc[_bus].mosi].FUNC_CTL = bus_desc[_bus].mosi_af;
    HPM_IOC->PAD[bus_desc[_bus].miso].FUNC_CTL = bus_desc[_bus].miso_af;
    HPM_IOC->PAD[bus_desc[_bus].sclk].FUNC_CTL = bus_desc[_bus].sclk_af | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);

    HPM_IOC->PAD[bus_desc[_bus].mosi].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1);
    HPM_IOC->PAD[bus_desc[_bus].miso].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1);
    HPM_IOC->PAD[bus_desc[_bus].sclk].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1);
    HPM_BIOC->PAD[bus_desc[_bus].mosi].FUNC_CTL = bus_desc[_bus].mosi_baf;
    HPM_BIOC->PAD[bus_desc[_bus].miso].FUNC_CTL = bus_desc[_bus].miso_baf;
    if (bus_desc[_bus].sclk_baf) {
        HPM_BIOC->PAD[bus_desc[_bus].sclk].FUNC_CTL = bus_desc[_bus].sclk_baf | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    }
    HPM_PIOC->PAD[bus_desc[_bus].mosi].FUNC_CTL = bus_desc[_bus].mosi_paf;
    HPM_PIOC->PAD[bus_desc[_bus].miso].FUNC_CTL = bus_desc[_bus].miso_paf;
    if (bus_desc[_bus].sclk_paf) {
        HPM_PIOC->PAD[bus_desc[_bus].sclk].FUNC_CTL = bus_desc[_bus].sclk_paf | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    }
    intc_m_enable_irq(bus_desc[_bus].dma_irqn);
    dma_tx_buffer = (uint32_t *)hpmhal.util->malloc_type(1024, AP_HAL::Util::MEM_DMA_SAFE);
    dma_rx_buffer = (uint32_t *)hpmhal.util->malloc_type(1024, AP_HAL::Util::MEM_DMA_SAFE);
}

SPIBus::~SPIBus()
{
    hpmhal.util->free_type(dma_tx_buffer, 1024, AP_HAL::Util::MEM_DMA_SAFE);
    hpmhal.util->free_type(dma_rx_buffer, 1024, AP_HAL::Util::MEM_DMA_SAFE);
}

SPIDevice::SPIDevice(SPIBus &_bus, SPIDeviceDesc &_device_desc)
    : bus(_bus)
    , device_desc(_device_desc)
{
#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    spi_timing_config_t timing_config = {0};
    spi_format_config_t format_config = {0};

    uint32_t spi_clock;
    set_device_bus(bus.bus);
    set_device_address(_device_desc.device);
    set_speed(AP_HAL::Device::SPEED_LOW);

    clock_add_to_group(bus_desc[_bus.bus].host_clk, 0);
    clock_add_to_group(clock_xdma, 0);
    clock_add_to_group(clock_hdma, 0);
    spi_clock = clock_get_frequency(bus_desc[_bus.bus].host_clk);

    spi_master_get_default_timing_config(&timing_config);
    timing_config.master_config.clk_src_freq_in_hz = spi_clock;
    timing_config.master_config.sclk_freq_in_hz = 1000000UL;
    if (status_success != spi_master_timing_init(bus_desc[_bus.bus].host, &timing_config)) {
        hpmhal.console->printf("SPI master timing init failed\n");
        return;
    }

    if (_device_desc.hspeed != _device_desc.lspeed) {
        timing_config.master_config.sclk_freq_in_hz = 10000000UL;
        if (status_success != spi_master_timing_init(bus_desc[_bus.bus].host, &timing_config)) {
            hpmhal.console->printf("SPI master timing init failed\n");
            return;
        }
    }
    /* set SPI format config for master */
    spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = 8;
    format_config.common_config.mode = spi_master_mode;
    format_config.common_config.cpol = spi_sclk_high_idle;
    format_config.common_config.cpha = spi_sclk_sampling_even_clk_edges;
    spi_format_init(bus_desc[_bus.bus].host, &format_config);
    pname = (char *)malloc(strlen(device_desc.name)+1);
    strcpy(pname, device_desc.name);
    hpmhal.console->printf("spi device constructed %s bus %d instance %lx\n", pname, _bus.bus, (uint32_t)bus_desc[_bus.bus].host);
}

SPIDevice::~SPIDevice()
{
    free(pname);
}


bool SPIDevice::set_speed(AP_HAL::Device::Speed _speed)
{
#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    speed = _speed;

    uint32_t spi_clock;
    spi_timing_config_t timing_config = {0};
    spi_clock = clock_get_frequency(bus_desc[bus.bus].host_clk);

    spi_master_get_default_timing_config(&timing_config);
    timing_config.master_config.clk_src_freq_in_hz = spi_clock;
    if (SPEED_HIGH == speed) {
        timing_config.master_config.sclk_freq_in_hz = device_desc.hspeed;
    } else {
        timing_config.master_config.sclk_freq_in_hz = device_desc.lspeed;
    }
    if (status_success != spi_master_timing_init(bus_desc[bus.bus].host, &timing_config)) {
        hpmhal.console->printf("SPI master timming init failed\n");
        while (1) {
            hpmhal.scheduler->delay(1000);
        }
    }
    return true;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (!send || !recv) {
        // simplest cases
        transfer_fullduplex(send, recv, recv_len?recv_len:send_len);
        return true;
    }
    uint8_t buf[send_len+recv_len];
    if (send_len > 0) {
        memcpy(buf, send, send_len);
    }
    if (recv_len > 0) {
        memset(&buf[send_len], 0, recv_len);
    }
    transfer_fullduplex(buf, buf, send_len+recv_len);
    if (recv_len > 0) {
        memcpy(recv, &buf[send_len], recv_len);
    }
    return true;
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    acquire_bus(true);
    uint32_t stat;
    if (send && recv && len) {
        spi_control_config_t control_config = {0};
        if (len <= 8) {
            stat = hpm_spi_transmit_receive_blocking(bus_desc[bus.bus].host, (uint8_t *)send, (uint8_t *)recv, len, HPM_SPI_TIMEOUT_VALUE);
        } else {
            SPIBusDesc *pbus_desc = &bus_desc[bus.bus];
            SPI_Type *instance = (SPI_Type *)(pbus_desc->host);
            memcpy(bus.dma_tx_buffer, send, len);
            if ((spi_is_active(instance) == true))
                return false;
            spi_disable_tx_dma(instance);
            spi_disable_rx_dma(instance);

            dmamux_config(HPM_DMAMUX, DMA_SOC_CHN_TO_DMAMUX_CHN(pbus_desc->dma, pbus_desc->dma_ch_tx), pbus_desc->dma_tx_src, true);
            dmamux_config(HPM_DMAMUX, DMA_SOC_CHN_TO_DMAMUX_CHN(pbus_desc->dma, pbus_desc->dma_ch_rx), pbus_desc->dma_rx_src, true);
            spi_master_get_default_control_config(&control_config);
            control_config.master_config.cmd_enable = false;
            control_config.master_config.addr_enable = false;
            control_config.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;
            control_config.common_config.tx_dma_enable = true;
            control_config.common_config.rx_dma_enable = true;
            control_config.common_config.trans_mode = spi_trans_write_read_together;
            control_config.common_config.data_phase_fmt = spi_single_io_mode;
            control_config.common_config.dummy_cnt = spi_dummy_count_1;

            stat = spi_setup_dma_transfer(instance,
                                &control_config,
                                0, 0,
                                len, len);
            if (stat != status_success) {
                while (1) {
                    hpmhal.scheduler->delay(1000);
                }
            }
            if (pbus_desc->dma == HPM_XDMA) {
                dmaDescriptors_xdma[pbus_desc->dma_ch_tx].bin_sem = get_binsemaphore();
            } else {
                dmaDescriptors_hdma[bus_desc[bus.bus].dma_ch_rx].bin_sem = get_binsemaphore();
            }
            dma_enable_channel_interrupt(pbus_desc->dma, bus_desc[bus.bus].dma_ch_rx, DMA_INTERRUPT_MASK_TERMINAL_COUNT);
            stat = spi_tx_trigger_dma(pbus_desc->dma,
                                    (uint32_t)bus_desc[bus.bus].dma_ch_tx,
                                    instance,
                                    (uint32_t)bus.dma_tx_buffer,
                                    DMA_TRANSFER_WIDTH_BYTE,
                                    len);
            if (stat != status_success) {
                hpmhal.console->printf("spi tx trigger dma failed\n");
                while (1) {
                    hpmhal.scheduler->delay(1000);
                }
            }
            stat = spi_rx_trigger_dma(pbus_desc->dma,
                                    (uint32_t)bus_desc[bus.bus].dma_ch_rx,
                                    instance,
                                    (uint32_t)bus.dma_rx_buffer,
                                    DMA_TRANSFER_WIDTH_BYTE,
                                    len);
            if (stat != status_success) {
                hpmhal.console->printf("spi rx trigger dma failed\n");
                while (1) {
                    hpmhal.scheduler->delay(1000);
                }
            }
            if (get_binsemaphore()->wait(100000) == false) {
                hpmhal.console->printf("SPI DMA RX timeout\n");
                while (1) {
                    hpmhal.scheduler->delay(1000);
                }
            }
            memcpy(recv, bus.dma_rx_buffer, len);
        }
    } else if (send != 0) {
        stat = hpm_spi_transmit_blocking(bus_desc[bus.bus].host, (uint8_t *)send, len, HPM_SPI_TIMEOUT_VALUE);
    } else if (recv != 0) {
        stat = hpm_spi_receive_blocking(bus_desc[bus.bus].host, recv, len, HPM_SPI_TIMEOUT_VALUE);
    } else {
        stat = 2;
    }
    if (stat != status_success) {
        hpmhal.console->printf("SPI transfer failed\n");
        while(1) {
            hpmhal.scheduler->delay(1000);
        }
    }
    acquire_bus(false);
    return true;
}

void SPIDevice::acquire_bus(bool acquire)
{
#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (acquire) {
        bool status = get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER);
        if (status) {
            gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(device_desc.cs), GPIO_GET_PIN_INDEX(device_desc.cs), 0);
        }
    } else {
        gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(device_desc.cs), GPIO_GET_PIN_INDEX(device_desc.cs), 1);
        bool status = get_semaphore()->give();
        (void)status;
    }
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return &bus.semaphore;
}
AP_HAL::BinarySemaphore *SPIDevice::get_binsemaphore()
{
#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return &bus.bin_sem;
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return bus.register_periodic_callback(period_usec, cb, this);
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

AP_HAL::OwnPtr<AP_HAL::SPIDevice> 
SPIDeviceManager::get_device(const char *name)
{
#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d %s\n", __PRETTY_FUNCTION__, __LINE__, name);
#endif
    uint8_t i;
    for (i = 0; i<ARRAY_SIZE(device_desc); i++) {
        if (strcmp(device_desc[i].name, name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE(device_desc)) {
        return nullptr;
    }
    SPIDeviceDesc &desc = device_desc[i];

#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d 222\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    // find the bus
    SPIBus *busp;
    for (busp = buses; busp; busp = (SPIBus *)busp->next) {
        if (busp->bus == desc.bus) {
            break;
        }
    }

#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d 333\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (busp == nullptr) {
        // create a new one
        busp = NEW_NOTHROW SPIBus(desc.bus);
        if (busp == nullptr) {
            return nullptr;
        }
        busp->next = buses;
        busp->bus = desc.bus;
        // deassert all CSes on the bus
        for (i = 0; i<ARRAY_SIZE(device_desc); i++) {
            SPIDeviceDesc &curr_desc = device_desc[i];
            if (desc.bus == curr_desc.bus) {
                HPM_BIOC->PAD[curr_desc.cs].FUNC_CTL = curr_desc.baf;
                HPM_PIOC->PAD[curr_desc.cs].FUNC_CTL = curr_desc.paf;
                HPM_IOC->PAD[curr_desc.cs].FUNC_CTL = 0;
                HPM_IOC->PAD[curr_desc.cs].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
                
                gpio_set_pin_output(HPM_GPIO0, GPIO_GET_PORT_INDEX(curr_desc.cs), GPIO_GET_PIN_INDEX(curr_desc.cs));
                gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(curr_desc.cs), GPIO_GET_PIN_INDEX(curr_desc.cs), 1);
            }
        }
        buses = busp;
    }

#ifdef SPIDEBUG
    hpmhal.console->printf("%s:%d 444\n", __PRETTY_FUNCTION__, __LINE__);
#endif

    return NEW_NOTHROW SPIDevice(*busp, desc);
}

