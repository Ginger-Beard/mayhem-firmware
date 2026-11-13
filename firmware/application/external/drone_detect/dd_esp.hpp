/*
 * Copyright (C) 2026 PortaPack Mayhem contributors
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

#ifndef __DD_ESP_HPP__
#define __DD_ESP_HPP__

#include <cstddef>
#include <cstdint>

#include "dd_model.hpp"

namespace ui::external_app::drone_detect {

/* Optional ESP32 add-on (htotoo's ESP32-Portapack firmware, I2C 0x51, driven
 * through i2cdev::I2cDev_PPmod) running its drone detector app (WiFi beacon
 * Remote ID, BLE Remote ID, drone maker OUIs). Protocol, from the module's
 * pp_handler.cpp / appmanager.cpp / ep_app_drone.cpp:
 *   0xA00C APPMGR   write u16 app id: start it (0 = stop), applied later in
 *                   the module's main loop; read u16: running app (10 = DRONE).
 *   0xA00D APPCMD   read u16[3] {count, 1, 0} from the drone app.
 *   0xA080 GET_ENTRY write u16 index, read one 44 byte drone_t; all zero
 *                   (source 0) past the end of the list.
 *   0xA081 CLEAR    empty the list (it never ages: max 24 entries).
 * Unknown commands answer a single 0xFF, so an old module firmware is told
 * apart by APPMGR never reporting 10 and APPCMD not reading {n, 1, 0}.
 * Replies are built in the module's I2C ISR, max 128 bytes.
 *
 * poll() runs from the UI thread on every frame sync and does at most one
 * short I2C transaction per call (~1-2 ms at 400 kHz for a 44 byte entry).
 * The list length is polled every 250 ms and the list re-read when it is;
 * only entries whose bytes changed since the last read are handed out (the
 * module keeps stale entries forever, so an unchanged entry is not a sign of
 * life). A future command returning raw 25 byte ODID messages would hand them
 * to Tracker::ingest_odid() instead.
 *
 * Recovery: a module reset stops its drone app (its APPCMD then answers 0xFF
 * or, with another app running, that app's counts) and NACKs the bus while it
 * boots; five failed transfers make the I2C device manager drop the device,
 * and inside an app nothing rescans the bus. So the running app id is
 * checked every 2 s, three failed polls in a row show "lost" (not a stale
 * count) and restart the drone app, and a missing device asks the manager for
 * a rescan (every 3 s, not while the bus is failing: a scan probes all 127
 * addresses, each into its timeout).
 *
 * Isolation: the module shares I2C0 with the Si5351 that makes the sample
 * clock. A transfer to it that ends in a bus fault (timeout, bus error, not a
 * plain NACK: I2C::stats()) stops all polling for 2 s, doubling up to 32 s
 * while it keeps happening ("bus err"), so a misbehaving module costs the
 * radio at most one faulted transfer per back-off. The device is held with
 * acquire_dev_by_model() over each command, so the I2C manager cannot delete
 * it in the middle of one. */
class EspFeed {
   public:
    enum class State : uint8_t {
        Off,
        NoModule,  // no PPmod device on the I2C bus
        Starting,  // asked the module to start its drone app, waiting
        NoApp,     // module firmware without the drone app
        Active,
        Lost,   // was active, stopped answering: restarting it
        Fault,  // a transfer to the module faulted the bus: backing off
    };

    static constexpr uint16_t kDroneAppId = 10;
    static constexpr size_t kMaxEntries = 24;

    void enable(bool on);
    /* Returns true and fills out when a new or changed entry was read.
     * now_ms: chTimeNow(). */
    bool poll(Summary& out, uint32_t now_ms);

    State state() const { return state_; }
    uint8_t count() const { return count_; }

   private:
    State state_{State::Off};
    uint32_t now_{0};
    uint32_t due_{0};       // next timed step
    uint32_t app_due_{0};   // next running app check (Active)
    uint32_t scan_due_{0};  // earliest next bus rescan
    uint8_t tries_{0};
    uint8_t fails_{0};
    uint8_t faults_{0};         // bus faults in a row (back-off exponent)
    bool bus_fault_{false};     // the last command() faulted the bus
    bool fault_active_{false};  // Fault entered from Active / Lost
    uint8_t count_{0};
    uint8_t next_{0};
    bool restore_{false};  // we started the drone app: put prev_app_ back on exit
    uint16_t prev_app_{0};
    uint16_t sums_[kMaxEntries]{};

    bool command(uint16_t cmd, const void* tx, size_t tx_len, void* rx, size_t rx_len);
    void enter(State s, uint32_t delay_ms);
    bool start_app();
    bool step(Summary& out);
};

} /* namespace ui::external_app::drone_detect */

#endif /*__DD_ESP_HPP__*/
