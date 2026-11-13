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

#include "i2c_pp.hpp"

namespace {

/* Recovery frame: START, a reserved address nothing answers, STOP. Nine
 * clocks, so a slave left mid-byte (driving SDA low, waiting for clocks)
 * gets to its ACK slot and lets go; NACK means the bus works again. */
constexpr i2caddr_t recovery_address = 0x7f;
constexpr size_t recovery_attempts = 9;
constexpr systime_t recovery_timeout = 2; /* ticks; the frame takes ~25 us at 400 kHz */

}  // namespace

void I2C::start(const I2CConfig& config) {
    i2cStart(_driver, &config);
}

void I2C::stop() {
    i2cStop(_driver);
}

/* Address-only frame (no data), the bus held by the caller. Bypasses the
 * HAL's I2C_READY check, as probe() always did, but not on a stopped driver:
 * its peripheral clock is off. */
msg_t I2C::probe_locked(i2caddr_t addr, systime_t timeout) {
    if (_driver->state == I2C_STOP) return RDY_TIMEOUT;
    chSysLock();
    _driver->errors = I2CD_NO_ERROR;
    _driver->state = I2C_ACTIVE_TX;
    const msg_t rdymsg = i2c_lld_master_transmit_timeout(_driver, addr, nullptr, 0, nullptr, 0, timeout);
    _driver->state = (rdymsg == RDY_TIMEOUT) ? I2C_LOCKED : I2C_READY;
    chSysUnlock();
    return rdymsg;
}

/* After every transfer, the bus held. A completed transfer and a plain NACK
 * (device absent or busy) leave the driver as it is: nothing changes on a
 * healthy bus. Anything else leaves the controller, and maybe the bus, in an
 * undefined state:
 *  - a timeout: the LLD aborted with STOP but the transfer may still be on
 *    the bus, its ISR still pointing at the caller's buffers, and the HAL
 *    refuses every later transfer (I2C_LOCKED, RDY_TIMEOUT at once) until
 *    the driver is stopped and started again (i2c_lld.c, "After a timeout
 *    the driver must be stopped and restarted");
 *  - a bus error, lost arbitration (there is no other master: a device drove
 *    SDA out of turn) or an unexpected controller state (RDY_RESET, no flag).
 * A stopped driver (sleep, clock changes) is not touched. */
void I2C::check(msg_t status, address_t addr, bool count_nack) {
    if (status == RDY_OK) {
        _stats.bus_ok = true;
        return;
    }
    if (_driver->state == I2C_STOP) return;
    const i2cflags_t flags = _driver->errors;
    if (status == RDY_RESET && flags == I2CD_ACK_FAILURE) {
        if (count_nack) _stats.nacks++;
        _stats.bus_ok = true;
        return;
    }
    _stats.faults++;
    if (status == RDY_TIMEOUT) _stats.timeouts++;
    _stats.last_fault_time = chTimeNow();
    _stats.last_fault_flags = flags | (status == RDY_TIMEOUT ? I2CD_TIMEOUT : 0);
    _stats.last_fault_addr = addr;
    recover();
}

/* Peripheral reset (i2cStop/i2cStart reset it through the RGU: pending
 * STA/STO and the ISR's view of the old transfer are gone), then the
 * recovery frame. I2C0 has dedicated open-drain pads (SFSI2C0, no GPIO
 * function), so SCL cannot be bit-banged: the controller's own clocks are
 * the only way to walk a stuck slave to its ACK slot. Bounded: at most
 * recovery_attempts x recovery_timeout (~18 ms), and one attempt (~2 ms)
 * while the bus has stayed dead since the last recovery. A device holding
 * SCL low cannot be helped from here; the bus stays failed (bus_ok false)
 * and every transfer fails within its timeout until the device lets go. */
void I2C::recover() {
    const I2CConfig* const config = _driver->config;
    _stats.recoveries++;
    const size_t attempts = _stats.bus_ok ? recovery_attempts : 1;
    for (size_t i = 0; i < attempts; i++) {
        i2cStop(_driver);
        i2cStart(_driver, config);
        const msg_t rdymsg = probe_locked(recovery_address, recovery_timeout);
        if (rdymsg == RDY_OK || (rdymsg == RDY_RESET && _driver->errors == I2CD_ACK_FAILURE)) {
            _stats.bus_ok = true;
            return;
        }
    }
    /* Leave the driver usable (I2C_READY): the next transfer tries again. */
    i2cStop(_driver);
    i2cStart(_driver, config);
    _stats.recovery_failed++;
    _stats.bus_ok = false;
}

bool I2C::probe(i2caddr_t addr, systime_t timeout) {
    i2cAcquireBus(_driver);
    const msg_t rdymsg = probe_locked(addr, timeout);
    check(rdymsg, addr, false);
    i2cReleaseBus(_driver);
    return (rdymsg == RDY_OK);
}

bool I2C::transfer(
    const address_t slave_address,
    const uint8_t* const data_tx,
    const size_t count_tx,
    uint8_t* const data_rx,
    const size_t count_rx,
    systime_t timeout) {
    i2cAcquireBus(_driver);
    const msg_t status = i2cMasterTransmitTimeout(
        _driver, slave_address, data_tx, count_tx, data_rx, count_rx, timeout);
    check(status, slave_address, true);
    i2cReleaseBus(_driver);
    return (status == RDY_OK);
}

bool I2C::receive(
    const address_t slave_address,
    uint8_t* const data,
    const size_t count,
    systime_t timeout) {
    i2cAcquireBus(_driver);
    const msg_t status = i2cMasterReceiveTimeout(
        _driver, slave_address, data, count, timeout);
    check(status, slave_address, true);
    i2cReleaseBus(_driver);
    return (status == RDY_OK);
}

bool I2C::transmit(
    const address_t slave_address,
    const uint8_t* const data,
    const size_t count,
    systime_t timeout) {
    return transfer(slave_address, data, count, NULL, 0, timeout);
}
