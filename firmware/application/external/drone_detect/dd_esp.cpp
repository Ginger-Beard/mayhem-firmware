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

#include "dd_esp.hpp"

#include "i2cdevmanager.hpp"
#include "i2cdev_ppmod.hpp"

#include <cstring>

namespace ui::external_app::drone_detect {

namespace {

constexpr uint16_t kCmdAppMgr = 0xA00C;
constexpr uint16_t kCmdAppCmd = 0xA00D;
constexpr uint16_t kCmdDroneGetEntry = 0xA080;
constexpr uint16_t kCmdDroneClear = 0xA081;
constexpr uint32_t kCountPollMs = 250;  /* list length: a new entry waits <= 0.25 s */
constexpr uint32_t kAppCheckMs = 2000;  /* running app id */
constexpr uint32_t kStartWaitMs = 1000; /* after asking the module to start the app */
constexpr uint32_t kNoModuleMs = 3000;
constexpr uint32_t kRescanMs = 3000;
constexpr uint8_t kStartTries = 5;
constexpr uint32_t kFaultBackoffMs = 2000; /* doubled per fault in a row, up to x16 */
constexpr uint8_t kFaultMaxShift = 4;

/* drone_t of the module's ep_app_drone.hpp. */
struct __attribute__((packed)) EspDrone {
    uint8_t source;                    // 1 BLE Remote ID, 2 WiFi Remote ID, 3 drone maker OUI / SSID
    char id[20];                       // UAS ID (19 chars kept), or the SSID
    int32_t lat, lon, op_lat, op_lon;  // 1e-7 degrees, 0 = unknown
    int8_t rssi;                       // dBm
    uint8_t mac[6];                    // BSSID or BLE address, MSB first
};
static_assert(sizeof(EspDrone) == 44, "drone_t layout");

uint16_t fletcher16(const uint8_t* p, size_t n) {
    uint16_t a = 0, b = 0;
    while (n--) {
        a = (a + *p++) % 255;
        b = (b + a) % 255;
    }
    return static_cast<uint16_t>((b << 8) | a) | 1; /* never 0, the "not read yet" mark */
}

}  // namespace

bool EspFeed::command(uint16_t cmd, const void* tx, size_t tx_len, void* rx, size_t rx_len) {
    if (bus_fault_) return false; /* one faulted transfer per poll(), not one per command */
    /* Held over the command: the I2C manager thread deletes a device after
     * five failed transfers, and a failing one here blocks for its timeout. */
    auto dev = static_cast<i2cdev::I2cDev_PPmod*>(i2cdev::I2CDevManager::acquire_dev_by_model(I2C_DEVMDL::I2CDECMDL_PPMOD));
    if (!dev) {
        /* Dropped after a few failed transfers (module reset): only a scan
         * finds it again, and apps run with autoscan off. */
        if (portapack::i2c0.stats().bus_ok && static_cast<int32_t>(now_ - scan_due_) >= 0) {
            i2cdev::I2CDevManager::manual_scan();
            scan_due_ = now_ + kRescanMs;
        }
        return false;
    }
    const uint32_t faults = portapack::i2c0.stats().faults;
    const bool ok = dev->custom_command(cmd, static_cast<const uint8_t*>(tx), tx_len, static_cast<uint8_t*>(rx), rx_len);
    i2cdev::I2CDevManager::release_dev(dev);
    if (ok)
        faults_ = 0;
    else if (portapack::i2c0.stats().faults != faults)
        bus_fault_ = true;
    return ok;
}

void EspFeed::enter(State s, uint32_t delay_ms) {
    state_ = s;
    due_ = now_ + delay_ms;
}

void EspFeed::enable(bool on) {
    if (on) {
        if (state_ == State::Off) {
            tries_ = 0;
            fails_ = 0;
            enter(State::Starting, 0);
        }
        return;
    }
    bus_fault_ = false;
    if (restore_) {
        command(kCmdAppMgr, &prev_app_, 2, nullptr, 0);
        restore_ = false;
    }
    state_ = State::Off;
    count_ = 0;
}

/* Asks the module to run its drone app (applied in its main loop). */
bool EspFeed::start_app() {
    const uint16_t id = kDroneAppId;
    if (!command(kCmdAppMgr, &id, 2, nullptr, 0)) return false;
    restore_ = true;
    return true;
}

bool EspFeed::poll(Summary& out, uint32_t now_ms) {
    now_ = now_ms;
    if (state_ == State::Off || state_ == State::NoApp) return false;
    if (state_ == State::Fault) {
        if (static_cast<int32_t>(now_ - due_) < 0) return false;
        /* Back-off over: one look, as after a module reset. */
        tries_ = 0;
        enter(fault_active_ ? State::Lost : State::NoModule, 0);
    }

    bus_fault_ = false;
    const bool got = step(out);
    if (bus_fault_) {
        /* Not a NACK: the transfer broke the bus the Si5351 shares. */
        fault_active_ = state_ == State::Active || state_ == State::Lost;
        if (faults_ <= kFaultMaxShift) faults_++;
        count_ = next_ = 0;
        enter(State::Fault, kFaultBackoffMs << (faults_ - 1));
        return false;
    }
    return got;
}

bool EspFeed::step(Summary& out) {
    if (state_ == State::Active && next_ < count_) {
        /* One list entry per frame. */
        const uint16_t index = next_++;
        EspDrone d{};
        if (!command(kCmdDroneGetEntry, &index, 2, &d, sizeof(d))) {
            /* Not one transfer per remaining entry into a dead bus: the
             * next length poll decides. */
            next_ = count_;
            return false;
        }
        if (d.source < 1 || d.source > 3) {
            next_ = count_; /* zeroed entry: end of the list */
            return false;
        }
        const uint16_t sum = fletcher16(reinterpret_cast<const uint8_t*>(&d), sizeof(d));
        if (sum == sums_[index]) return false;
        sums_[index] = sum;

        out.source = d.source == 1 ? SRC_ESP_BLE : SRC_ESP_WIFI;
        out.kind = d.source == 3 ? Kind::WifiOui : Kind::RemoteId;
        out.rssi = d.rssi;
        memcpy(out.mac, d.mac, 6);
        out.lat = d.lat;
        out.lon = d.lon;
        out.op_lat = d.op_lat;
        out.op_lon = d.op_lon;
        /* Same clean-up as odid_lite applies to a Basic ID, so the UAS IDs of
         * both paths compare equal. */
        size_t n = 0;
        for (; n < sizeof(d.id) && d.id[n]; n++) {
            const char ch = d.id[n];
            out.id[n] = (ch >= 0x20 && ch <= 0x7E) ? ch : '?';
        }
        while (n && out.id[n - 1] == ' ') n--;
        memset(out.id + n, 0, sizeof(out.id) - n);
        return true;
    }

    if (static_cast<int32_t>(now_ - due_) < 0) return false;

    if (state_ == State::Active) {
        if (static_cast<int32_t>(now_ - app_due_) >= 0) {
            /* Is the drone app still the one running? After a module reset
             * none is; another app's APPCMD would answer with its counts. */
            app_due_ = now_ + kAppCheckMs;
            uint16_t app = 0xFFFF;
            if (command(kCmdAppMgr, nullptr, 0, &app, 2) && app != kDroneAppId) {
                tries_ = 0;
                start_app();
                enter(State::Lost, kStartWaitMs);
                return false;
            }
        }
        /* Every kCountPollMs: how long is the list? */
        uint16_t status[3] = {0xFFFF, 0, 0};
        if (!command(kCmdAppCmd, nullptr, 0, status, sizeof(status)) || status[1] != 1 || status[0] > kMaxEntries) {
            /* Three in a row (0.75 s): say so, and restart the app. */
            if (++fails_ >= 3) {
                tries_ = 0;
                enter(State::Lost, 0);
            } else {
                due_ = now_ + kCountPollMs;
            }
            return false;
        }
        fails_ = 0;
        if (status[0] == kMaxEntries && count_ == kMaxEntries) {
            /* Full: the module would drop new drones. Our copies age on their own. */
            command(kCmdDroneClear, nullptr, 0, nullptr, 0);
            memset(sums_, 0, sizeof(sums_));
            status[0] = 0;
        }
        count_ = static_cast<uint8_t>(status[0]);
        next_ = 0;
        due_ = now_ + kCountPollMs;
        return false;
    }

    /* NoModule, Starting or Lost. */
    uint16_t app = 0xFFFF;
    if (!command(kCmdAppMgr, nullptr, 0, &app, 2)) {
        /* Lost stays lost (and retries every second) until it answers. */
        enter(state_ == State::Lost ? State::Lost : State::NoModule, state_ == State::Lost ? kStartWaitMs : kNoModuleMs);
        return false;
    }
    if (app == kDroneAppId) {
        count_ = next_ = fails_ = 0;
        app_due_ = now_ + kAppCheckMs;
        enter(State::Active, 0);
        return false;
    }
    const bool was_active = state_ == State::Lost;
    if (tries_ == 0 && !restore_) prev_app_ = app < 64 ? app : 0; /* whatever ran before us */
    if (++tries_ > kStartTries && !was_active) {
        /* Firmware without the drone app. Starting an unknown app stops the
         * running one on firmware that has the app manager: put it back. */
        enable(false);
        state_ = State::NoApp;
        return false;
    }
    start_app();
    enter(was_active ? State::Lost : State::Starting, kStartWaitMs);
    return false;
}

} /* namespace ui::external_app::drone_detect */
