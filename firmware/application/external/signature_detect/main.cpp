/*
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

#include "ui.hpp"
#include "ui_signature_detect.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::signature_detect {
void initialize_app(ui::NavigationView& nav) {
    nav.push<SignatureDetectView>();
}
}  // namespace ui::external_app::signature_detect

extern "C" {

__attribute__((section(".external_app.app_signature_detect.application_information"), used)) application_information_t _application_information_signature_detect = {
    /*.memory_location = */ (uint8_t*)0x00000000,
    /*.externalAppEntry = */ ui::external_app::signature_detect::initialize_app,
    /*.header_version = */ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,

    /*.app_name = */ "Sig Detect",
    /*.bitmap_data = */ {0x00, 0x00, 0x0E, 0x70, 0x1F, 0xF8, 0x0E, 0x70, 0x04, 0x20, 0x07, 0xE0, 0x0F, 0xF0, 0x0F, 0xF0, 0x07, 0xE0, 0x04, 0x20, 0x0E, 0x70, 0x1F, 0xF8, 0x0E, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /*.icon_color = */ ui::Color::orange().v,
    /*.menu_location = */ app_location_t::RX,
    /*.desired_menu_position = */ -1,

#ifdef PRALINE
    /*.m4_app_tag = portapack::spi_flash::image_tag_signature_detect */ {'P', 'S', 'D', 'T'},
#else
    // HackRF Pro only: the app is a stub elsewhere and carries no baseband.
    /*.m4_app_tag = */ {0, 0, 0, 0},
#endif
    /*.m4_app_offset = */ 0x00000000,  // will be filled at compile time
};
}
