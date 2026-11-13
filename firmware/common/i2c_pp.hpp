/*
 * Copyright (C) 2014 Jared Boone, ShareBrained Technology, Inc.
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef __I2C_PP_H__
#define __I2C_PP_H__

#include <cstdint>

#include "ch.h"
#include "hal.h"

struct I2CClockConfig {
    float clock_source_f;
    float bus_f;
    float high_period_ns;

    static constexpr float period_ns(const float f) {
        return 1e9 / f;
    }

    constexpr uint32_t i2c_period_count() const {
        return period_ns(bus_f) / period_ns(clock_source_f) + 0.5f;
    }

    constexpr uint32_t i2c_high_count() const {
        return high_period_ns / period_ns(clock_source_f) + 0.5f;
    }

    constexpr uint32_t i2c_low_count() const {
        return i2c_period_count() - i2c_high_count();
    }
};

/* Bus health, for diagnostics (USB shell "sysinfo", app status lines). Written
 * with the bus held, read without a lock. A plain NACK is not a fault. */
struct I2CStats {
    uint32_t faults{0};           // transfers ended by a timeout, bus error, lost arbitration or unexpected bus state
    uint32_t timeouts{0};         // ... of which timeouts
    uint32_t nacks{0};            // transfers (not probes) NACKed by the addressed device
    uint32_t recoveries{0};       // controller resets after a fault
    uint32_t recovery_failed{0};  // ... after which the bus still did not work (a device holds SCL or SDA low)
    systime_t last_fault_time{0};
    i2cflags_t last_fault_flags{0};  // I2CD_* error flags of the last fault, I2CD_TIMEOUT for a timeout
    uint8_t last_fault_addr{0};
    bool bus_ok{true};  // false after a failed recovery, true again after the next completed transfer
};

class I2C {
   public:
    using address_t = uint8_t;

    /* Used where the caller passes no timeout. The slaves on these buses
     * (Si5351, codecs, battery gauges, sensors) answer at once, and the
     * longest default-timeout transfer is 71 bytes (~1.6 ms at 400 kHz), so
     * this never expires on a healthy bus; it only turns a held bus into an
     * error instead of a thread blocked forever with the bus mutex held. */
    static constexpr systime_t timeout_default = MS2ST(20);

    /* For callers that want to give up early: the transfer time of `bytes`
     * (plus the address byte) at 100 kHz, a quarter of the bus rate, plus
     * 5 ticks for tick granularity and interrupt latency. */
    static constexpr systime_t timeout_for(const size_t bytes) {
        return 5 + ((bytes + 1) * 9 * CH_FREQUENCY + 99999) / 100000;
    }

    constexpr I2C(I2CDriver* const driver)
        : _driver(driver) {
    }

    void start(const I2CConfig& config);
    void stop();
    bool probe(i2caddr_t addr, systime_t timeout = timeout_default);
    bool receive(
        const address_t slave_address,
        uint8_t* const data,
        const size_t count,
        const systime_t timeout = timeout_default);

    bool transmit(
        const address_t slave_address,
        const uint8_t* const data,
        const size_t count,
        const systime_t timeout = timeout_default);

    const I2CStats& stats() const { return _stats; }

   private:
    I2CDriver* const _driver;
    I2CStats _stats{};

    msg_t probe_locked(i2caddr_t addr, systime_t timeout);
    void check(msg_t status, address_t addr, bool count_nack);
    void recover();

    bool transfer(
        const address_t slave_address,
        const uint8_t* const data_tx,
        const size_t count_tx,
        uint8_t* const data_rx,
        const size_t count_rx,
        const systime_t timeout);
};

#endif /*__I2C_PP_H__*/
