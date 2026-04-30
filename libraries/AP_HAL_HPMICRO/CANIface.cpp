/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Pavel Kirienko
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

#include "AP_HAL_HPM.h"
#include "HAL_HPM_Class.h"
#ifdef HPMSOC_HAS_HPMSDK_MCAN
#include "hpm_mcan_drv.h"
#else
#include "hpm_can_drv.h"
#endif
#include "hpm_gpio_drv.h"
#include "hpm_soc.h"
#include "hpm_ioc_regs.h"
#if HAL_NUM_CAN_IFACES
#include <cassert>
#include <cstring>
#include <AP_Math/AP_Math.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/ExpandingString.h>

#include "CANIface.h"

struct CanBusDesc can_bus_desc[HAL_NUM_CAN_IFACES] = { HAL_CAN_BUS_DESC_LIST };
// FIFO elements are spaced at 18 words
#define FDCAN_FRAME_BUFFER_SIZE 18

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

#define STR(x) #x
#define XSTR(x) STR(x)

using namespace HPMicro;

#if HAL_CANMANAGER_ENABLED
#define Debug(fmt, args...) do { AP::can().log_text(AP_CANManager::LOG_DEBUG, "CANFDIface", fmt, ##args); } while (0)
#else
#define Debug(fmt, args...)
#endif

// const CANIface::CanType* CANIface::Can[HAL_NUM_CAN_IFACES] = { HAL_CAN_BASE_LIST };
const clock_name_t CANIface::clock_can[HAL_NUM_CAN_IFACES] = { HAL_CAN_CLOCK_LIST };
static HPMicro::CANIface* can_ifaces[HAL_NUM_CAN_IFACES];

uint8_t CANIface::next_interface;

// mapping from logical interface to physical. First physical is 0, first logical is 0
static constexpr uint8_t can_interfaces[HAL_NUM_CAN_IFACES] = { HAL_CAN_INTERFACE_LIST };

// mapping from physical interface back to logical. First physical is 0, first logical is 0
static constexpr int8_t can_iface_to_idx[3] = { HAL_CAN_INTERFACE_REV_LIST };

#define REG_SET_TIMEOUT 250 // if it takes longer than 250ms for setting a register we have failed

static inline bool driver_initialised(uint8_t iface_index)
{
    if (can_ifaces[iface_index] == nullptr) {
        return false;
    }
    return true;
}

static inline void handleCANInterrupt(uint8_t phys_index)
{
    const int8_t iface_index = can_iface_to_idx[phys_index];
    if (iface_index < 0 || iface_index >= HAL_NUM_CAN_IFACES) {
        return;
    }
    CAN_Type *can = (CAN_Type *)can_bus_desc[iface_index].base;
    uint8_t tx_rx_flags = can_get_tx_rx_flags(can);
    uint8_t error_flags = can_get_error_interrupt_flags(can);
    /* Handle TX/RX flags */
    if ((tx_rx_flags & CAN_EVENT_RECEIVE) != 0U) {

        while (can_is_data_available_in_receive_buffer(can)) {
            
            can_ifaces[iface_index]->handleRxInterrupt(0);
        }
        can_clear_tx_rx_flags(can, CAN_EVENT_RECEIVE);
    }

    if ((tx_rx_flags & (CAN_EVENT_TX_PRIMARY_BUF | CAN_EVENT_TX_SECONDARY_BUF )) != 0) {
        can_clear_tx_rx_flags(can, CAN_EVENT_TX_PRIMARY_BUF);
        uint64_t timestamp_us = AP_HAL::micros64();
        if (timestamp_us > 0) {
            timestamp_us--;
        }
        can_ifaces[iface_index]->handleTxCompleteInterrupt(timestamp_us, tx_rx_flags);
    }

    if ((tx_rx_flags & CAN_EVENT_ERROR) != 0) {
        can_clear_tx_rx_flags(can, CAN_EVENT_ERROR);
        can_ifaces[iface_index]->handleBusOffInterrupt();
    }

    /* Handle error flags */
    if ((error_flags & (CAN_ERRINT_EPIE_MASK | CAN_ERRINT_ALIE_MASK | CAN_ERRINT_BEIE_MASK)) != 0U) {
        can_clear_error_interrupt_flags(can, error_flags);
        can_ifaces[iface_index]->handleBusOffInterrupt();
    }
    if (!driver_initialised(iface_index)) {
        //Just return
        return;
    }
    can_ifaces[iface_index]->pollErrorFlagsFromISR();
}

uint32_t CANIface::FDCANMessageRAMOffset_ = 0;

CANIface::CANIface(uint8_t index) :
    rx_bytebuffer_((uint8_t*)rx_buffer, sizeof(rx_buffer)),
    rx_queue_(&rx_bytebuffer_),
    self_index_(index)
{
    if (index >= HAL_NUM_CAN_IFACES) {
         AP_HAL::panic("Bad CANIface index.");
    } else {
        can_ = (CanType *)can_bus_desc[index].base;
    }
    HPM_IOC->PAD[can_bus_desc[index].rx_ioc].FUNC_CTL = can_bus_desc[index].rx_af;
    HPM_IOC->PAD[can_bus_desc[index].tx_ioc].FUNC_CTL = can_bus_desc[index].tx_af;
    HPM_BIOC->PAD[can_bus_desc[index].tx_ioc].FUNC_CTL = can_bus_desc[index].tx_baf;
    if (can_bus_desc[index].rx_baf) {
        HPM_BIOC->PAD[can_bus_desc[index].rx_ioc].FUNC_CTL = can_bus_desc[index].rx_baf;
    }
    if (can_bus_desc[index].tx_baf) {
        HPM_BIOC->PAD[can_bus_desc[index].tx_ioc].FUNC_CTL = can_bus_desc[index].tx_baf;
    }
    if (can_bus_desc[index].rx_paf) {
        HPM_PIOC->PAD[can_bus_desc[index].rx_ioc].FUNC_CTL = can_bus_desc[index].rx_paf;
    }
    if (can_bus_desc[index].tx_paf) {
        HPM_PIOC->PAD[can_bus_desc[index].tx_ioc].FUNC_CTL = can_bus_desc[index].tx_paf;
    }
    clock_set_source_divider(can_bus_desc[index].clock, clk_src_pll1_clk1, 5);
    clock_add_to_group(can_bus_desc[index].clock, 0);
    intc_m_enable_irq(can_bus_desc[index].irqn);
}

// constructor suitable for array
CANIface::CANIface() :
    CANIface(next_interface++)
{}

void CANIface::handleBusOffInterrupt()
{
    _detected_bus_off = true;
}

int16_t CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                       CanIOFlags flags)
{
    if (!initialised_) {
        return -1;
    }

    stats.tx_requests++;
    if (frame.isErrorFrame() || (frame.dlc > 8 && !frame.isCanFDFrame()) ||
        frame.dlc > 15) {
        stats.tx_rejected++;
        return -1;
    }

    {
        CriticalSectionLocker lock;

        /*
         * Seeking for an empty slot
         */
        uint8_t index;
        if (!can_is_primary_transmit_buffer_full(can_)) {
            index = 0;
        } else {
            index = 1;
        }

        // Copy Frame to RAM
        // Calculate Tx element address

        can_transmit_buf_t tx_buf;
        tx_buf.buffer[0] = 0;
        tx_buf.buffer[1] = 0;

        //Setup Frame ID
        tx_buf.id = frame.id;
        if (frame.isExtended()) {
            tx_buf.extend_id = true;
        } else {
            tx_buf.extend_id = false;
        }
        if (frame.isRemoteTransmissionRequest()) {
            tx_buf.remote_frame = true;
        } else {
            tx_buf.remote_frame = false;
        }
        //Write Data Length Code, and Message Marker
        tx_buf.dlc =  frame.dlc;

        if (frame.isCanFDFrame()) {
            tx_buf.canfd_frame = true;
            stats.fdf_tx_requests++;
            pending_tx_[index].canfd_frame = true;
        } else {
            tx_buf.canfd_frame = false;
            pending_tx_[index].canfd_frame = false;
        }

        // Write Frame to the message RAM
        const uint8_t data_length = AP_HAL::CANFrame::dlcToDataLength(frame.dlc);
        uint32_t *data_ptr = (uint32_t *)&tx_buf.data[0];
        for (uint8_t i = 0; i < (data_length+3)/4; i++) {
            data_ptr[i] = frame.data_32[i];
        }

        //Registering the pending transmission so we can track its deadline and loopback it as needed
        pending_tx_[index].deadline       = tx_deadline;
        pending_tx_[index].frame          = frame;
        pending_tx_[index].loopback       = (flags & AP_HAL::CANIface::Loopback) != 0;
        pending_tx_[index].abort_on_error = (flags & AP_HAL::CANIface::AbortOnError) != 0;
        pending_tx_[index].index          = index;
        // setup frame initial state
        pending_tx_[index].aborted        = false;
        pending_tx_[index].setup          = true;
        pending_tx_[index].pushed         = false;
        hpm_stat_t status;
        if (index == 0) {
            status = can_send_high_priority_message_nonblocking(can_, &tx_buf);
        } else {
            status = can_send_message_nonblocking(can_, &tx_buf);
        }
        if (status != status_success) {
            stats.tx_rejected++;
            return -1;
        }
    }

    // also send on MAVCAN, but don't consider it an error if we can't get the MAVCAN out
    AP_HAL::CANIface::send(frame, tx_deadline, flags);

    return 1;
}

int16_t CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us, CanIOFlags& out_flags)
{
    {
        CriticalSectionLocker lock;
        CanRxItem rx_item;
        if (!rx_queue_.pop(rx_item) || !initialised_) {
            return 0;
        }
        out_frame    = rx_item.frame;
        out_timestamp_us = rx_item.timestamp_us;
        out_flags    = rx_item.flags;
    }

    return AP_HAL::CANIface::receive(out_frame, out_timestamp_us, out_flags);
}

bool CANIface::configureFilters(const CanFilterConfig* filter_configs,
                                uint16_t num_configs)
{
    initialised_ = true;
    return true;
}

uint16_t CANIface::getNumFilters() const
{
    return 1;
}

bool CANIface::clock_init_ = false;
bool CANIface::init(const uint32_t bitrate, const uint32_t fdbitrate, const OperatingMode mode)
{
    uint32_t can_src_clk_freq;
    Debug("Bitrate %lu mode %d", static_cast<unsigned long>(bitrate), static_cast<int>(mode));
    if (self_index_ > HAL_NUM_CAN_IFACES) {
        Debug("CAN drv init failed");
        return false;
    }
    if (can_ifaces[self_index_] == nullptr) {
        can_ifaces[self_index_] = this;
#if !defined(HAL_BOOTLOADER_BUILD)
        AP_HAL::get_HAL_mutable().can[self_index_] = this;
#endif
    }

    bitrate_ = bitrate;
    mode_ = mode;
    //Only do it once
    //Doing it second time will reset the previously initialised bus
    if (!clock_init_) {
        CriticalSectionLocker lock;
        can_src_clk_freq = board_init_can_clock((CAN_Type *)can_bus_desc[self_index_].base);
        clock_init_ = true;
    } else {
        can_src_clk_freq = clock_get_frequency(clock_can[self_index_]);
    }

    // Setup FDCAN for configuration mode and disable all interrupts
    {
        CriticalSectionLocker lock;
        can_config_t can_config;
        can_get_default_config(&can_config);
        can_config.baudrate = bitrate;
        can_config.mode = can_mode_normal;
        can_config.irq_txrx_enable_mask = CAN_EVENT_RECEIVE/* | CAN_EVENT_TX_PRIMARY_BUF | CAN_EVENT_TX_SECONDARY_BUF*/;
        board_init_can(can_);
        hpm_stat_t status = can_init(can_, &can_config, can_src_clk_freq);
        if (status != status_success) {
            hpmhal.console->printf("CAN initialization failed, error code: %ld\n", status);
            return false;
        }
    }

    /*
     * IRQ
     */
    if (!irq_init_) {
        CriticalSectionLocker lock;
        switch (can_interfaces[self_index_]) {
        case 0:
            intc_m_enable_irq_with_priority(IRQn_CAN0, 1);
            break;
        case 1:
            intc_m_enable_irq_with_priority(IRQn_CAN1, 1);
            break;
        case 2:
            intc_m_enable_irq_with_priority(IRQn_CAN2, 1);
            break;
        case 3:
            intc_m_enable_irq_with_priority(IRQn_CAN3, 1);
            break;
        }
        irq_init_ = true;
    }

    /*
     * Object state - interrupts are disabled, so it's safe to modify it now
     */
    rx_queue_.clear();
    for (uint32_t i=0; i < NumTxMailboxes; i++) {
        pending_tx_[i] = CanTxItem();
    }
    peak_tx_mailbox_index_ = 0;
    had_activity_ = false;

    _bitrate = bitrate;

    // Reset Bus Off
    _detected_bus_off = false;
    // If mode is Filtered then we finish the initialisation in configureFilter method
    // otherwise we finish here
    if (mode != FilteredMode) {
        //initialised
        initialised_ = true;
    }
    return true;
}

void CANIface::clear_rx()
{
    CriticalSectionLocker lock;
    rx_queue_.clear();
}

void CANIface::handleTxCompleteInterrupt(const uint64_t timestamp_us, uint32_t mask)
{
    uint32_t index;
    if (mask & CAN_EVENT_TX_PRIMARY_BUF) {
        index = 0;
    } else if (mask & CAN_EVENT_TX_SECONDARY_BUF) {
        index = 1;
    } else {
        return;
    }
    if (!pending_tx_[index].pushed) {
        stats.tx_success++;
        stats.last_transmit_us = timestamp_us;
        if (pending_tx_[index].canfd_frame) {
            stats.fdf_tx_success++;
        }
        pending_tx_[index].pushed = true;
    }

    if (pending_tx_[index].loopback && had_activity_) {
        CanRxItem rx_item;
        rx_item.frame = pending_tx_[index].frame;
        rx_item.timestamp_us = timestamp_us;
        rx_item.flags = AP_HAL::CANIface::Loopback;
        add_to_rx_queue(rx_item);
    }
    stats.num_events++;
    if (sem_handle != nullptr) {
        sem_handle->signal_ISR();
    }
}

bool CANIface::readRxFIFO(uint8_t fifo_index)
{
    uint64_t timestamp_us = AP_HAL::micros64();
   
    hpm_stat_t status = status_invalid_argument;
    can_receive_buf_t rx_buf;
    memset(&rx_buf, 0, sizeof(rx_buf));
    status = can_read_received_message(can_, &rx_buf);
    if (status != status_success) {
        return false;
    }
    // Read the frame contents
    AP_HAL::CANFrame frame {};
    if ((rx_buf.extend_id) == 0) {
        //Standard ID
        frame.id = ((rx_buf.id & STID_MASK) >> 18) & AP_HAL::CANFrame::MaskStdID;
    } else {
        //Extended ID
        frame.id = (rx_buf.id & EXID_MASK) & AP_HAL::CANFrame::MaskExtID;
        frame.id |= AP_HAL::CANFrame::FlagEFF;
    }

    if ((rx_buf.remote_frame) != 0) {
        frame.id |= AP_HAL::CANFrame::FlagRTR;
    }

    if (rx_buf.canfd_frame) {
        frame.setCanFD(true);
        stats.fdf_rx_received++;
    } else {
        frame.setCanFD(false);
    }

    frame.dlc = rx_buf.dlc;
    uint8_t *data = (uint8_t*)&rx_buf.buffer[2];

    for (uint8_t i = 0; i < AP_HAL::CANFrame::dlcToDataLength(frame.dlc); i++) {
        frame.data[i] = data[i];
    }

    /*
     * Store with timeout into the FIFO buffer
     */

    CanRxItem rx_item;
    rx_item.frame = frame;
    rx_item.timestamp_us = timestamp_us;
    rx_item.flags = 0;
    if (add_to_rx_queue(rx_item)) {
        stats.rx_received++;
    } else {
        stats.rx_overflow++;
    }
    return true;
}

void CANIface::handleRxInterrupt(uint8_t fifo_index)
{
    if (readRxFIFO(fifo_index) != true) {
        return;
    }
    had_activity_ = true;
    stats.num_events++;
    if (sem_handle != nullptr) {
        sem_handle->signal_ISR();
    }
}

/**
 * This method is used to count errors and abort transmission on error if necessary.
 * This functionality used to be implemented in the SCE interrupt handler, but that approach was
 * generating too much processing overhead, especially on disconnected interfaces.
 *
 * Should be called from RX ISR, TX ISR, and select(); interrupts must be enabled.
 */
void CANIface::pollErrorFlagsFromISR()
{
}

void CANIface::pollErrorFlags()
{
    CriticalSectionLocker cs_locker;
    pollErrorFlagsFromISR();
}

bool CANIface::canAcceptNewTxFrame() const
{
    if (can_is_primary_transmit_buffer_full(can_)) {
        return false;
    }
    if (can_is_secondary_transmit_buffer_full(can_)) {
        return false;
    }

    return true;
}

/**
 * Total number of hardware failures and other kinds of errors (e.g. queue overruns).
 * May increase continuously if the interface is not connected to the bus.
 */
uint32_t CANIface::getErrorCount() const
{
    CriticalSectionLocker lock;
    return stats.num_busoff_err +
           stats.rx_errors +
           stats.rx_overflow +
           stats.tx_rejected +
           stats.tx_abort +
           stats.tx_timedout;
}

bool CANIface::set_event_handle(AP_HAL::BinarySemaphore *handle)
{
    sem_handle = handle;
    return true;
}

bool CANIface::isRxBufferEmpty() const
{
    CriticalSectionLocker lock;
    return rx_queue_.available() == 0;
}

void CANIface::clearErrors()
{
    if (_detected_bus_off) {
        //Try Recovering from BusOff
        //While in Bus off mode the CAN Peripheral is put
        //into INIT mode, when we ask Peripheral to get out
        //of INIT mode, the bit stream processor (BSP) synchronizes
        //itself to the data transfer on the CAN bus by
        //waiting for the occurrence of a sequence of 11 consecutive
        //recessive bits (Bus_Idle) before it can take part in bus
        //activities and start the message transfer
        stats.num_busoff_err++;
        _detected_bus_off = false;
    }
}

void CANIface::checkAvailable(bool& read, bool& write, const AP_HAL::CANFrame* pending_tx) const
{
    write = false;
    read = !isRxBufferEmpty();
    if (pending_tx != nullptr) {
        write = canAcceptNewTxFrame();
    }
}

bool CANIface::select(bool &read, bool &write,
                      const AP_HAL::CANFrame* pending_tx,
                      uint64_t blocking_deadline)
{
    const bool in_read = read;
    const bool in_write= write;
    uint64_t time = AP_HAL::micros64();

    if (!read && !write) {
        //invalid request
        return false;
    }

    pollErrorFlags();
    clearErrors();

    checkAvailable(read, write, pending_tx);          // Check if we already have some of the requested events
    if ((read && in_read) || (write && in_write)) {
        return true;
    }
    while (time < blocking_deadline) {
        if (sem_handle == nullptr) {
            break;
        }
        IGNORE_RETURN(sem_handle->wait(blocking_deadline - time)); // Block until timeout expires or any iface updates
        checkAvailable(read, write, pending_tx);  // Check what we got
        if ((read && in_read) || (write && in_write)) {
            return true;
        }
        time = AP_HAL::micros64();
    }
    return false;
}

#if !defined(HAL_BOOTLOADER_BUILD)
void CANIface::get_stats(ExpandingString &str)
{
    CriticalSectionLocker lock;
    // str.printf("------- Clock Config -------\n"
    //            "CAN_CLK_FREQ:   %luMHz\n"
    //            "Std Timings: bitrate=%lu presc=%u\n"
    //            "sjw=%u bs1=%u bs2=%u sample_point=%f%%\n"
    //            "FD Timings:  bitrate=%lu presc=%u\n"
    //            "sjw=%u bs1=%u bs2=%u sample_point=%f%%\n"
    //            "------- CAN Interface Stats -------\n"
    //            "tx_requests:    %lu\n"
    //            "tx_rejected:    %lu\n"
    //            "tx_overflow:    %lu\n"
    //            "tx_success:     %lu\n"
    //            "tx_timedout:    %lu\n"
    //            "tx_abort:       %lu\n"
    //            "rx_received:    %lu\n"
    //            "rx_overflow:    %lu\n"
    //            "rx_errors:      %lu\n"
    //            "num_busoff_err: %lu\n"
    //            "num_events:     %lu\n"
    //            "ECR:            %lx\n"
    //            "fdf_rx:         %lu\n"
    //            "fdf_tx_req:     %lu\n"
    //            "fdf_tx:         %lu\n",
    //            STM32_FDCANCLK/1000000UL,
    //            _bitrate, unsigned(timings.prescaler),
    //            unsigned(timings.sjw), unsigned(timings.bs1),
    //            unsigned(timings.bs2), timings.sample_point_permill/10.0f,
    //            _fdbitrate, unsigned(fdtimings.prescaler),
    //            unsigned(fdtimings.sjw), unsigned(fdtimings.bs1),
    //            unsigned(fdtimings.bs2), fdtimings.sample_point_permill/10.0f,
    //            stats.tx_requests,
    //            stats.tx_rejected,
    //            stats.tx_overflow,
    //            stats.tx_success,
    //            stats.tx_timedout,
    //            stats.tx_abort,
    //            stats.rx_received,
    //            stats.rx_overflow,
    //            stats.rx_errors,
    //            stats.num_busoff_err,
    //            stats.num_events,
    //            stats.ecr,
    //            stats.fdf_rx_received,
    //            stats.fdf_tx_requests,
    //            stats.fdf_tx_success);
}
#endif

/*
 * Interrupt handlers
 */
extern "C"
{
#ifdef HAL_CAN_IFACE0_ENABLE
    // FDCAN1
    SDK_DECLARE_EXT_ISR_M(IRQn_CAN0, can0_irq_handler);
    void can0_irq_handler(void)
    {
        handleCANInterrupt(0);
    }
#endif

#ifdef HAL_CAN_IFACE1_ENABLE
    // FDCAN2
    SDK_DECLARE_EXT_ISR_M(IRQn_CAN1, can1_irq_handler);
    void can1_irq_handler(void)
    {
        handleCANInterrupt(1);
    }
#endif

#ifdef HAL_CAN_IFACE2_ENABLE
    // FDCAN3
    SDK_DECLARE_EXT_ISR_M(IRQn_CAN2, can2_irq_handler);
    void can2_irq_handler(void)
    {
        handleCANInterrupt(2);
    }
#endif

#ifdef HAL_CAN_IFACE3_ENABLE
    // FDCAN4
    SDK_DECLARE_EXT_ISR_M(IRQn_CAN3, can3_irq_handler);
    void can3_irq_handler(void)
    {
        handleCANInterrupt(3);
    }
#endif
    
} // extern "C"

#endif //HAL_NUM_CAN_IFACES