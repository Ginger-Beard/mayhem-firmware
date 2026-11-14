/*
 * Copyright (C) 2015 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2016 Furrtek
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

#ifndef __MESSAGE_H__
#define __MESSAGE_H__

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <functional>
#include <algorithm>

#include "baseband_packet.hpp"

#include "adsb_frame.hpp"
#include "ert_packet.hpp"
#include "pocsag_packet.hpp"
#include "flex_defs.hpp"
#include "aprs_packet.hpp"
#include "sonde_packet.hpp"
#include "tpms_packet.hpp"
#include "jammer.hpp"
#include "dsp_fir_taps.hpp"
#include "dsp_iir.hpp"
#include "fifo.hpp"
#include "spec_an_shared.hpp"
#include "sigdet_wifi_rid.hpp"
#include "sigdet_fsk.hpp"

#include "utility.hpp"

#include "ch.h"

class Message {
   public:
    static constexpr size_t MAX_SIZE = 512;

    enum class ID : uint32_t {
        /* Assign consecutive IDs. IDs are used to index array. */
        RSSIStatistics = 0,
        BasebandStatistics = 1,
        ChannelStatistics = 2,
        DisplayFrameSync = 3,
        AudioStatistics = 4,
        Shutdown = 5,
        TPMSPacket = 6,
        ACARSPacket = 7,
        AISPacket = 8,
        ERTPacket = 9,
        SondePacket = 10,
        UpdateSpectrum = 11,
        NBFMConfigure = 12,
        WFMConfigure = 13,
        AMConfigure = 14,
        ChannelSpectrumConfig = 15,
        SpectrumStreamingConfig = 16,
        DisplaySleep = 17,
        CaptureConfig = 18,
        CaptureThreadDone = 19,
        ReplayConfig = 20,
        ReplayThreadDone = 21,
        AFSKRxConfigure = 22,
        StatusRefresh = 23,
        SampleRateConfig = 24,
        BTLERxConfigure = 25,
        NRFRxConfigure = 26,
        TXProgress = 27,
        Retune = 28,
        TonesConfigure = 29,
        AFSKTxConfigure = 30,
        PitchRSSIConfigure = 31,
        OOKConfigure = 32,
        RDSConfigure = 33,
        AudioTXConfig = 34,
        POCSAGConfigure = 35,
        DTMFTXConfig = 36,
        ADSBConfigure = 37,
        JammerConfigure = 38,
        WidebandSpectrumConfig = 39,
        FSKConfigure = 40,
        SSTVConfigure = 41,
        SigGenConfig = 42,
        SigGenTone = 43,
        POCSAGPacket = 44,
        ADSBFrame = 45,
        AFSKData = 46,
        TestAppPacket = 47,
        RequestSignal = 48,
        FIFOData = 49,
        AudioLevelReport = 50,
        CodedSquelch = 51,
        AudioSpectrum = 52,
        APRSPacket = 53,
        APRSRxConfigure = 54,
        SpectrumPainterBufferRequestConfigure = 55,
        SpectrumPainterBufferResponseConfigure = 56,
        POCSAGStats = 57,
        FSKRxConfigure = 58,
        BlePacket = 58,
        BTLETxConfigure = 59,
        SubGhzFPRxConfigure = 60,
        WeatherData = 61,
        SubGhzDData = 62,
        GPSPosData = 63,
        OrientationData = 64,
        EnvironmentData = 65,
        AudioBeep = 66,
        PocsagTosend = 67,
        BatteryStateData = 68,
        ProtoViewData = 69,
        FreqChangeCommand = 70,
        I2CDevListChanged = 71,
        LightData = 72,
        WeFaxRxConfigure = 73,
        WeFaxRxStatusData = 74,
        WeFaxRxImageData = 75,
        WFMAMConfigure = 76,
        NoaaAptRxConfigure = 77,
        NoaaAptRxStatusData = 78,
        NoaaAptRxImageData = 79,
        FSKPacket = 80,
        EPIRBPacket = 81,
        FlexPacket = 82,
        FlexStats = 83,
        FlexConfigure = 84,
        FlexDebug = 85,
        SSTVRXConfigure = 86,
        SSTVRXProgress = 87,
        SSTVRXPhaseSlant = 88,
        SSTVRXCalibration = 89,
        SubCarData = 90,
        MorseRXData = 91,
        MorseRXfreq = 92,
        MorseRXConfig = 93,
        TXDisabled = 94,
        MorseTXConfigure = 95,
        MorseTXkey = 96,
        StreamTXConfiguration = 97,
        RTTYData = 98,
        NotificationData = 99,
        TimeSinkConfig = 100,
        EPIRBTXData = 101,
        P25TxConfigure = 102,
        ToneDetectData = 103,
        ToneDetectConfig = 104,
        FlexTosend = 105,
        EPIRBRXConfig = 106,
        VorRxConfigure = 107,
        VorRxStatusData = 108,
        VorTxConfigure = 109,
        HunterConfig = 110,
        HunterTrigger = 111,
        HunterStop = 112,
        TetraBsch = 113,
        TetraDnb = 114,
        AudioDDCConfig = 115,
        RxFs4Config = 116,
        FT8Packet = 117,
        FT8RxStatus = 118,
        RdsData = 119,
        SpecAnConfig = 120,
        SpecAnCaptured = 121,
        SpecAnSlice = 122,
        WMBusPacketMessageID = 123,
        VDL2Frame = 124,
        SubGhzChatPacket = 125,
        SignatureDetectConfig = 126,
        SignatureDetectRid = 127,
        SignatureDetectStats = 128,
        SignatureDetectPatternHit = 129,
        SignatureDetectWifi = 130,
        SignatureDetectDroneId = 131,
        SignatureDetectMeter = 132,
        SignatureDetectFsk = 133,
        GpsRxConfig = 134,
        GpsRxStatus = 135,
        MAX
    };

    constexpr Message(
        ID id)
        : id{id} {
    }

    const ID id;
};

struct RSSIStatistics {
    uint32_t accumulator{0};
    uint8_t min{0};
    uint8_t max{0};
    uint16_t count{0};
};

class RSSIStatisticsMessage : public Message {
   public:
    constexpr RSSIStatisticsMessage(
        const RSSIStatistics& statistics)
        : Message{ID::RSSIStatistics},
          statistics{statistics} {
    }

    RSSIStatistics statistics;
};

struct BasebandStatistics {
    uint32_t idle_ticks{0};
    uint32_t main_ticks{0};
    uint32_t rssi_ticks{0};
    uint32_t baseband_ticks{0};
    bool saturation{false};
};

class BasebandStatisticsMessage : public Message {
   public:
    constexpr BasebandStatisticsMessage(
        const BasebandStatistics& statistics)
        : Message{ID::BasebandStatistics},
          statistics{statistics} {
    }

    BasebandStatistics statistics;
};

struct ChannelStatistics {
    int32_t max_db;
    size_t count;

    constexpr ChannelStatistics(
        int32_t max_db = -120,
        size_t count = 0)
        : max_db{max_db},
          count{count} {
    }
};

class ChannelStatisticsMessage : public Message {
   public:
    constexpr ChannelStatisticsMessage(
        const ChannelStatistics& statistics)
        : Message{ID::ChannelStatistics},
          statistics{statistics} {
    }

    ChannelStatistics statistics;
};

class DisplayFrameSyncMessage : public Message {
   public:
    constexpr DisplayFrameSyncMessage()
        : Message{ID::DisplayFrameSync} {
    }
};

struct AudioStatistics {
    int32_t rms_db;
    int32_t max_db;
    size_t count;

    constexpr AudioStatistics()
        : rms_db{-120},
          max_db{-120},
          count{0} {
    }

    constexpr AudioStatistics(
        int32_t rms_db,
        int32_t max_db,
        size_t count)
        : rms_db{rms_db},
          max_db{max_db},
          count{count} {
    }
};

class DisplaySleepMessage : public Message {
   public:
    constexpr DisplaySleepMessage()
        : Message{ID::DisplaySleep} {
    }
};

class StatusRefreshMessage : public Message {
   public:
    constexpr StatusRefreshMessage()
        : Message{ID::StatusRefresh} {
    }
};

class AudioStatisticsMessage : public Message {
   public:
    constexpr AudioStatisticsMessage(
        const AudioStatistics& statistics)
        : Message{ID::AudioStatistics},
          statistics{statistics} {
    }

    AudioStatistics statistics;
};

class SpectrumStreamingConfigMessage : public Message {
   public:
    enum class Mode : uint32_t {
        Stopped = 0,
        Running = 1,
    };

    constexpr SpectrumStreamingConfigMessage(
        Mode mode)
        : Message{ID::SpectrumStreamingConfig},
          mode{mode} {
    }

    Mode mode{Mode::Stopped};
};

// Application sample-rate translation, independent of PRALINE's AFE shift.
enum class RxFs4Direction : uint8_t { Down,
                                      Up };

class RxFs4ConfigMessage : public Message {
   public:
    constexpr RxFs4ConfigMessage(RxFs4Direction direction)
        : Message{ID::RxFs4Config}, direction{direction} {}

    const RxFs4Direction direction;
};

class AudioDDCConfigMessage : public Message {
   public:
    constexpr AudioDDCConfigMessage(int32_t frequency)
        : Message{ID::AudioDDCConfig},
          frequency{frequency} {
    }

    int32_t frequency{0};
};

class WidebandSpectrumConfigMessage : public Message {
   public:
    constexpr WidebandSpectrumConfigMessage(
        size_t sampling_rate,
        size_t trigger)
        : Message{ID::WidebandSpectrumConfig},
          sampling_rate{sampling_rate},
          trigger{trigger} {
    }

    size_t sampling_rate{0};
    size_t trigger{0};
};

class TimeSinkConfigMessage : public Message {
   public:
    constexpr TimeSinkConfigMessage(
        size_t sampling_rate,
        size_t trigger)
        : Message{ID::TimeSinkConfig},
          sampling_rate{sampling_rate},
          trigger{trigger} {
    }

    size_t sampling_rate{0};
    size_t trigger{0};
};

struct AudioSpectrum {
    std::array<uint8_t, 128> db{{0}};
    // uint32_t sampling_rate { 0 };
};

class AudioSpectrumMessage : public Message {
   public:
    constexpr AudioSpectrumMessage(
        AudioSpectrum* data)
        : Message{ID::AudioSpectrum},
          data{data} {
    }

    AudioSpectrum* data{nullptr};
};

struct ChannelSpectrum {
    std::array<uint8_t, 256> db{{0}};
    uint32_t sampling_rate{0};
    int32_t channel_filter_offset{0};
    int32_t channel_filter_low_frequency{0};
    int32_t channel_filter_high_frequency{0};
    int32_t channel_filter_transition{0};
};

using ChannelSpectrumFIFO = FIFO<ChannelSpectrum>;

class ChannelSpectrumConfigMessage : public Message {
   public:
    static constexpr size_t fifo_k = 2;

    constexpr ChannelSpectrumConfigMessage(
        ChannelSpectrumFIFO* fifo)
        : Message{ID::ChannelSpectrumConfig},
          fifo{fifo} {
    }

    ChannelSpectrumFIFO* fifo{nullptr};
};

class AISPacketMessage : public Message {
   public:
    constexpr AISPacketMessage(
        const baseband::Packet& packet)
        : Message{ID::AISPacket},
          packet{packet} {
    }

    baseband::Packet packet;
};

class EPIRBPacketMessage : public Message {
   public:
    constexpr EPIRBPacketMessage(
        const baseband::Packet& packet)
        : Message{ID::EPIRBPacket},
          packet{packet} {
    }

    baseband::Packet packet;
};

class EPIRBRXConfig : public Message {
   public:
    constexpr EPIRBRXConfig()
        : Message{ID::EPIRBRXConfig} {
    }
    bool spectrum_on = false;
    bool audio_on = true;
    uint8_t squelch{50};
};

/* One decoded FT8 transmission, already unpacked to text by the baseband. */
class FT8PacketMessage : public Message {
   public:
    /* Longest text an FT8 payload unpacks to: callsign[13], space, callsign[13], space,
     * report[6], terminator. */
    static constexpr size_t text_length = 35;

    constexpr FT8PacketMessage(
        const char* message_text,
        int16_t audio_frequency)
        : Message{ID::FT8Packet},
          text{},
          frequency{audio_frequency} {
        size_t i = 0;
        for (; i < text_length - 1 && message_text[i] != '\0'; i++)
            text[i] = message_text[i];
        text[i] = '\0';
    }

    char text[text_length];
    /* Audio frequency inside the 200-2500 Hz passband. Every station on the band shares
     * one dial and picks its own slot, so this is what places a decode in the passband. */
    int16_t frequency;
};

/* Sent once per 15 s slot so the UI can tell the user whether the receiver has found
 * the slot boundary; an unsynchronised FT8 receiver decodes nothing and looks identical
 * to a dead band. */
class FT8RxStatusMessage : public Message {
   public:
    enum class SyncState : uint8_t {
        Searching = 0,  // Nothing above the noise; hunting for the slot boundary
        Heard = 1,      // An FT8 transmission is in the passband but has not decoded
        Syncing = 2,    // Boundary found, walking the sub-symbol phase
        Locked = 3,     // Tracking the slot
    };

    constexpr FT8RxStatusMessage(
        SyncState sync_state,
        uint8_t decodes)
        : Message{ID::FT8RxStatus},
          state{sync_state},
          decode_count{decodes} {
    }

    SyncState state;
    uint8_t decode_count;
};

class TPMSPacketMessage : public Message {
   public:
    constexpr TPMSPacketMessage(
        const tpms::SignalType signal_type,
        const baseband::Packet& packet)
        : Message{ID::TPMSPacket},
          signal_type{signal_type},
          packet{packet} {
    }

    tpms::SignalType signal_type;
    baseband::Packet packet;
};

class POCSAGPacketMessage : public Message {
   public:
    constexpr POCSAGPacketMessage(
        const pocsag::POCSAGPacket& packet)
        : Message{ID::POCSAGPacket},
          packet{packet} {
    }

    pocsag::POCSAGPacket packet;
};

class POCSAGStatsMessage : public Message {
   public:
    constexpr POCSAGStatsMessage(
        uint32_t current_bits,
        uint8_t current_frames,
        bool has_sync,
        uint16_t baud_rate)
        : Message{ID::POCSAGStats},
          current_bits{current_bits},
          current_frames{current_frames},
          has_sync{has_sync},
          baud_rate{baud_rate} {
    }

    uint32_t current_bits = 0;
    uint8_t current_frames = 0;
    bool has_sync = false;
    uint16_t baud_rate = 0;
};

class ACARSPacketMessage : public Message {
   public:
    constexpr ACARSPacketMessage()
        : Message{ID::ACARSPacket} {}
    uint8_t msg_len = 0;
    char message[250] = {0};  // contains the whole packet
    uint8_t crc[2] = {0};
    uint8_t state = 0;  // for debug
};

class ADSBFrameMessage : public Message {
   public:
    constexpr ADSBFrameMessage(
        const adsb::ADSBFrame& frame,
        const uint32_t amp)
        : Message{ID::ADSBFrame},
          frame{frame},
          amp(amp) {
    }

    adsb::ADSBFrame frame;
    uint32_t amp;
};

class AFSKDataMessage : public Message {
   public:
    constexpr AFSKDataMessage(
        const bool is_data,
        const uint32_t value)
        : Message{ID::AFSKData},
          is_data{is_data},
          value{value} {
    }

    bool is_data;
    uint32_t value;
};

struct ADV_PDU_PAYLOAD_TYPE_0_2_4_6 {
    uint8_t Data[31];
};

struct ADV_PDU_PAYLOAD_TYPE_1_3 {
    uint8_t A1[6];
};

struct ADV_PDU_PAYLOAD_TYPE_5 {
    uint8_t AdvA[6];
    uint8_t AA[4];
    uint32_t CRCInit;
    uint8_t WinSize;
    uint16_t WinOffset;
    uint16_t Interval;
    uint16_t Latency;
    uint16_t Timeout;
    uint8_t ChM[5];
    uint8_t Hop;
    uint8_t SCA;
};

struct BlePacketData {
    int max_dB;
    uint8_t type;
    uint8_t size;
    uint8_t macAddress[6];
    uint8_t data[40];
    uint8_t dataLen;
};

struct FskPacketData {
    int8_t real;
    int8_t imag;
    int max_dB;
    uint8_t data[512];
    uint16_t dataLen;
    uint64_t syncWord;
    float power;
    float frequency_offset_hz;
};

class BLEPacketMessage : public Message {
   public:
    constexpr BLEPacketMessage(
        BlePacketData* packet)
        : Message{ID::BlePacket},
          packet{packet} {
    }

    BlePacketData* packet{nullptr};
};

class FSKRxPacketMessage : public Message {
   public:
    constexpr FSKRxPacketMessage(
        FskPacketData* packet)
        : Message{ID::FSKPacket},
          packet{packet} {
    }

    FskPacketData* packet{nullptr};
};

class CodedSquelchMessage : public Message {
   public:
    constexpr CodedSquelchMessage(
        const uint32_t value)
        : Message{ID::CodedSquelch},
          value{value} {
    }

    uint32_t value;
};

class ShutdownMessage : public Message {
   public:
    constexpr ShutdownMessage()
        : Message{ID::Shutdown} {
    }
};

class ERTPacketMessage : public Message {
   public:
    constexpr ERTPacketMessage(
        const ert::Packet::Type type,
        const baseband::Packet& packet)
        : Message{ID::ERTPacket},
          type{type},
          packet{packet} {
    }

    ert::Packet::Type type;

    baseband::Packet packet;
};

class SondePacketMessage : public Message {
   public:
    constexpr SondePacketMessage(
        const sonde::Packet::Type type,
        const baseband::Packet& packet)
        : Message{ID::SondePacket},
          type{type},
          packet{packet} {
    }

    sonde::Packet::Type type;

    baseband::Packet packet;
};

class TestAppPacketMessage : public Message {
   public:
    constexpr TestAppPacketMessage(
        const baseband::Packet& packet)
        : Message{ID::TestAppPacket},
          packet{packet} {
    }

    baseband::Packet packet;
};

class UpdateSpectrumMessage : public Message {
   public:
    constexpr UpdateSpectrumMessage()
        : Message{ID::UpdateSpectrum} {
    }
};

class NBFMConfigureMessage : public Message {
   public:
    constexpr NBFMConfigureMessage(
        const fir_taps_real<24> decim_0_filter,
        const fir_taps_real<32> decim_1_filter,
        const fir_taps_real<32> channel_filter,
        const size_t channel_decimation,
        const size_t deviation,
        const iir_biquad_config_t audio_hpf_config,
        const iir_biquad_config_t audio_deemph_config,
        const uint8_t squelch_level)
        : Message{ID::NBFMConfigure},
          decim_0_filter(decim_0_filter),
          decim_1_filter(decim_1_filter),
          channel_filter(channel_filter),
          channel_decimation{channel_decimation},
          deviation{deviation},
          audio_hpf_config(audio_hpf_config),
          audio_deemph_config(audio_deemph_config),
          squelch_level(squelch_level) {
    }

    const fir_taps_real<24> decim_0_filter;
    const fir_taps_real<32> decim_1_filter;
    const fir_taps_real<32> channel_filter;
    const size_t channel_decimation;
    const size_t deviation;
    const iir_biquad_config_t audio_hpf_config;
    const iir_biquad_config_t audio_deemph_config;
    const uint8_t squelch_level;
};

class WFMConfigureMessage : public Message {
   public:
    constexpr WFMConfigureMessage(
        const fir_taps_real<24> decim_0_filter,
        const fir_taps_real<16> decim_1_filter,
        const fir_taps_real<64> audio_filter,
        const size_t deviation,
        const iir_biquad_config_t audio_hpf_config,
        const iir_biquad_config_t audio_deemph_config)
        : Message{ID::WFMConfigure},
          decim_0_filter(decim_0_filter),
          decim_1_filter(decim_1_filter),
          audio_filter(audio_filter),
          deviation{deviation},
          audio_hpf_config(audio_hpf_config),
          audio_deemph_config(audio_deemph_config) {
    }

    const fir_taps_real<24> decim_0_filter;
    const fir_taps_real<16> decim_1_filter;
    const fir_taps_real<64> audio_filter;
    const size_t deviation;
    const iir_biquad_config_t audio_hpf_config;
    const iir_biquad_config_t audio_deemph_config;
};

class WFMAMConfigureMessage : public Message {
   public:
    constexpr WFMAMConfigureMessage(
        const fir_taps_real<24> decim_0_filter,
        const fir_taps_real<32> decim_1_filter,
        const fir_taps_real<64> audio_filter,
        const size_t deviation,
        const iir_biquad_config_t audio_hpf_config,
        const iir_biquad_config_t audio_deemph_config)
        : Message{ID::WFMAMConfigure},
          decim_0_filter(decim_0_filter),
          decim_1_filter(decim_1_filter),
          audio_filter(audio_filter),
          deviation{deviation},
          audio_hpf_config(audio_hpf_config),
          audio_deemph_config(audio_deemph_config) {
    }

    const fir_taps_real<24> decim_0_filter;
    const fir_taps_real<32> decim_1_filter;
    const fir_taps_real<64> audio_filter;
    const size_t deviation;
    const iir_biquad_config_t audio_hpf_config;
    const iir_biquad_config_t audio_deemph_config;
};

class AMConfigureMessage : public Message {
   public:
    enum class Modulation : int32_t {
        DSB = 0,
        SSB = 1,
        SSB_FM = 2,  // Added new for RX Wefax mode,  to demodulate APT signal ,FM modulated inside audio subcarrier tones, and then up broadcasted in SSB USB .
    };
    enum class Zoom_waterfall : size_t {
        ZOOM_x_1 = 1,
        ZOOM_x_2 = 2,
        ZOOM_x_3 = 4,
        ZOOM_x_4 = 8,
    };

    constexpr AMConfigureMessage(
        const fir_taps_real<24> decim_0_filter,
        const fir_taps_real<32> decim_1_filter,
        const fir_taps_real<32> decim_2_filter,
        const fir_taps_complex<64> channel_filter,
        const Modulation modulation,
        const iir_biquad_config_t audio_hpf_lpf_config,
        const size_t channel_spectrum_decimation_factor,
        const uint8_t squelch_level = 0)

        : Message{ID::AMConfigure},
          decim_0_filter(decim_0_filter),
          decim_1_filter(decim_1_filter),
          decim_2_filter(decim_2_filter),
          channel_filter(channel_filter),
          modulation{modulation},
          audio_hpf_lpf_config(audio_hpf_lpf_config),
          channel_spectrum_decimation_factor(channel_spectrum_decimation_factor),
          squelch_level(squelch_level) {
    }

    const fir_taps_real<24> decim_0_filter;
    const fir_taps_real<32> decim_1_filter;
    const fir_taps_real<32> decim_2_filter;
    const fir_taps_complex<64> channel_filter;
    const Modulation modulation;
    const iir_biquad_config_t audio_hpf_lpf_config;
    const size_t channel_spectrum_decimation_factor;
    const uint8_t squelch_level;  // AM channel-power squelch threshold (0 = off)
};

// TODO: Put this somewhere else, or at least the implementation part.
class StreamBuffer {
    uint8_t* data_;
    size_t used_;
    size_t capacity_;

   public:
    constexpr StreamBuffer(
        void* const data = nullptr,
        const size_t capacity = 0)
        : data_{static_cast<uint8_t*>(data)},
          used_{0},
          capacity_{capacity} {
    }

    size_t write(const void* p, const size_t count) {
        const auto copy_size = std::min(capacity_ - used_, count);
        memcpy(&data_[used_], p, copy_size);
        used_ += copy_size;
        return copy_size;
    }

    size_t read(void* p, const size_t count) {
        const auto copy_size = std::min(used_, count);
        memcpy(p, &data_[capacity_ - used_], copy_size);
        used_ -= copy_size;
        return copy_size;
    }

    bool is_full() const {
        return used_ >= capacity_;
    }

    bool is_empty() const {
        return used_ == 0;
    }

    void* data() const {
        return data_;
    }

    size_t size() const {
        return used_;
    }

    size_t capacity() const {
        return capacity_;
    }

    void set_size(const size_t value) {
        used_ = value;
    }

    void empty() {
        used_ = 0;
    }
};

struct CaptureConfig {
    const size_t write_size;
    const size_t buffer_count;
    uint64_t baseband_bytes_received;
    uint64_t baseband_bytes_dropped;
    FIFO<StreamBuffer*>* fifo_buffers_empty;
    FIFO<StreamBuffer*>* fifo_buffers_full;

    constexpr CaptureConfig(
        const size_t write_size,
        const size_t buffer_count)
        : write_size{write_size},
          buffer_count{buffer_count},
          baseband_bytes_received{0},
          baseband_bytes_dropped{0},
          fifo_buffers_empty{nullptr},
          fifo_buffers_full{nullptr} {
    }

    size_t dropped_percent() const {
        if (baseband_bytes_dropped == 0) {
            return 0;
        } else {
            const size_t percent = baseband_bytes_dropped * 100U / baseband_bytes_received;
            return std::max(1U, percent);
        }
    }
};

class CaptureConfigMessage : public Message {
   public:
    constexpr CaptureConfigMessage(
        CaptureConfig* const config)
        : Message{ID::CaptureConfig},
          config{config} {
    }

    CaptureConfig* const config;
};

struct ReplayConfig {
    const size_t read_size;
    const size_t buffer_count;
    uint64_t baseband_bytes_received;
    FIFO<StreamBuffer*>* fifo_buffers_empty;
    FIFO<StreamBuffer*>* fifo_buffers_full;
    // Opt-in only for native C8 direct IQ Replay. Audio/GPS default unchanged.
    const bool input_is_c8;

    constexpr ReplayConfig(
        const size_t read_size,
        const size_t buffer_count,
        bool input_is_c8 = false)
        : read_size{read_size},
          buffer_count{buffer_count},
          baseband_bytes_received{0},
          fifo_buffers_empty{nullptr},
          fifo_buffers_full{nullptr},
          input_is_c8{input_is_c8} {
    }
};

class ReplayConfigMessage : public Message {
   public:
    constexpr ReplayConfigMessage(
        ReplayConfig* const config)
        : Message{ID::ReplayConfig},
          config{config} {
    }

    ReplayConfig* const config;
};

class TXProgressMessage : public Message {
   public:
    constexpr TXProgressMessage()
        : Message{ID::TXProgress} {
    }

    uint32_t progress = 0;
    bool done = false;
};

class AFSKRxConfigureMessage : public Message {
   public:
    constexpr AFSKRxConfigureMessage(
        const uint32_t baudrate,
        const uint32_t word_length,
        const uint32_t trigger_value,
        const bool trigger_word)
        : Message{ID::AFSKRxConfigure},
          baudrate(baudrate),
          word_length(word_length),
          trigger_value(trigger_value),
          trigger_word(trigger_word) {
    }

    const uint32_t baudrate;
    const uint32_t word_length;
    const uint32_t trigger_value;
    const bool trigger_word;
};

class APRSRxConfigureMessage : public Message {
   public:
    constexpr APRSRxConfigureMessage(
        const uint32_t baudrate)
        : Message{ID::APRSRxConfigure},
          baudrate(baudrate) {
    }

    const uint32_t baudrate;
};

class BTLERxConfigureMessage : public Message {
   public:
    constexpr BTLERxConfigureMessage(
        const uint8_t channel_number)
        : Message{ID::BTLERxConfigure},
          channel_number(channel_number) {
    }
    const uint8_t channel_number;
};

class BTLETxConfigureMessage : public Message {
   public:
    constexpr BTLETxConfigureMessage(
        const uint8_t channel_number,
        char* macAddress,
        char* advertisementData,
        const uint8_t pduType)
        : Message{ID::BTLETxConfigure},
          channel_number(channel_number),
          macAddress(macAddress),
          advertisementData(advertisementData),
          pduType(pduType) {
    }
    const uint8_t channel_number;
    char* macAddress;
    char* advertisementData;
    const uint8_t pduType;
};

class NRFRxConfigureMessage : public Message {
   public:
    constexpr NRFRxConfigureMessage(
        const uint32_t baudrate,
        const uint32_t word_length,
        const uint32_t trigger_value,
        const bool trigger_word)
        : Message{ID::NRFRxConfigure},
          baudrate(baudrate),
          word_length(word_length),
          trigger_value(trigger_value),
          trigger_word(trigger_word) {
    }
    const uint32_t baudrate;
    const uint32_t word_length;
    const uint32_t trigger_value;
    const bool trigger_word;
};

class PitchRSSIConfigureMessage : public Message {
   public:
    constexpr PitchRSSIConfigureMessage(
        const bool enabled,
        const int32_t rssi)
        : Message{ID::PitchRSSIConfigure},
          enabled(enabled),
          rssi(rssi) {
    }

    const bool enabled;
    const int32_t rssi;
};

class TonesConfigureMessage : public Message {
   public:
    constexpr TonesConfigureMessage(
        const uint32_t fm_delta,
        const uint32_t pre_silence,
        const uint16_t tone_count,
        const bool dual_tone,
        const bool audio_out)
        : Message{ID::TonesConfigure},
          fm_delta(fm_delta),
          pre_silence(pre_silence),
          tone_count(tone_count),
          dual_tone(dual_tone),
          audio_out(audio_out) {
    }

    const uint32_t fm_delta;
    const uint32_t pre_silence;
    const uint16_t tone_count;
    const bool dual_tone;
    const bool audio_out;
};

class RDSConfigureMessage : public Message {
   public:
    constexpr RDSConfigureMessage(
        const uint16_t length)
        : Message{ID::RDSConfigure},
          length(length) {
    }

    const uint16_t length = 0;
};

class RetuneMessage : public Message {
   public:
    constexpr RetuneMessage()
        : Message{ID::Retune} {
    }

    int64_t freq = 0;
    uint32_t range = 0;
};

/* Oversample/Interpolation sample rate multipliers. */
enum class OversampleRate : uint8_t {
    /* Use either to indicate there's no oversampling needed. */
    None = 1,
    x1 = None,

    /* Replay-only x2; Capture's historical selection is unchanged. */
    x2 = 2,

    /* Oversample rate of 4 times the sample rate. */
    x4 = 4,

    /* Oversample rate of 8 times the sample rate. */
    x8 = 8,

    /* Oversample rate of 16 times the sample rate. */
    x16 = 16,

    /* Oversample rate of 32 times the sample rate. */
    x32 = 32,

    /* Oversample rate of 64 times the sample rate. */
    x64 = 64,
};

class SampleRateConfigMessage : public Message {
   public:
    constexpr SampleRateConfigMessage(
        uint32_t sample_rate,
        OversampleRate oversample_rate)
        : Message{ID::SampleRateConfig},
          sample_rate(sample_rate),
          oversample_rate(oversample_rate) {
    }

    const uint32_t sample_rate = 0;
    const OversampleRate oversample_rate = OversampleRate::None;
};

class AudioLevelReportMessage : public Message {
   public:
    constexpr AudioLevelReportMessage()
        : Message{ID::AudioLevelReport} {
    }

    uint32_t value = 0;
};

class AudioTXConfigMessage : public Message {
   public:
    constexpr AudioTXConfigMessage(
        const uint32_t divider,
        const float deviation_hz,
        const float audio_gain,
        const uint8_t audio_shift_bits_s16,
        const uint8_t bits_per_sample,
        const uint32_t tone_key_delta,
        const float tone_key_mix_weight,
        const bool am_enabled,
        const bool dsb_enabled,
        const bool usb_enabled,
        const bool lsb_enabled)
        : Message{ID::AudioTXConfig},
          divider(divider),
          deviation_hz(deviation_hz),
          audio_gain(audio_gain),
          audio_shift_bits_s16(audio_shift_bits_s16),
          bits_per_sample(bits_per_sample),
          tone_key_delta(tone_key_delta),
          tone_key_mix_weight(tone_key_mix_weight),
          am_enabled(am_enabled),
          dsb_enabled(dsb_enabled),
          usb_enabled(usb_enabled),
          lsb_enabled(lsb_enabled) {
    }

    const uint32_t divider;
    const float deviation_hz;
    const float audio_gain;
    const uint8_t audio_shift_bits_s16;
    const uint8_t bits_per_sample;
    const uint32_t tone_key_delta;
    const float tone_key_mix_weight;
    const bool am_enabled;
    const bool dsb_enabled;
    const bool usb_enabled;
    const bool lsb_enabled;
};

class SigGenConfigMessage : public Message {
   public:
    constexpr SigGenConfigMessage(
        const uint32_t bw,
        const uint32_t shape,
        const uint32_t duration)
        : Message{ID::SigGenConfig},
          bw(bw),
          shape(shape),
          duration(duration) {
    }

    const uint32_t bw;
    const uint32_t shape;
    const uint32_t duration;
};

class SigGenToneMessage : public Message {
   public:
    constexpr SigGenToneMessage(
        const uint32_t tone_delta)
        : Message{ID::SigGenTone},
          tone_delta(tone_delta) {
    }

    const uint32_t tone_delta;
};

class EPIRBTXDataMessage : public Message {
   public:
    static constexpr uint8_t max_len = 32;
    constexpr EPIRBTXDataMessage()
        : Message{ID::EPIRBTXData} {
    }
    bool mode_406 = true;
    uint8_t data[max_len]{0};
    uint8_t data_len = 0;
    uint32_t pre_count = 0;
    uint32_t post_count = 0;
};

class P25TxConfigureMessage : public Message {
   public:
    constexpr P25TxConfigureMessage()
        : Message{ID::P25TxConfigure} {
    }
    uint16_t frame_length{0};
};

class AFSKTxConfigureMessage : public Message {
   public:
    constexpr AFSKTxConfigureMessage(
        const uint32_t samples_per_bit,
        const uint32_t phase_inc_mark,
        const uint32_t phase_inc_space,
        const uint8_t repeat,
        const uint32_t fm_delta,
        const uint8_t symbol_count)
        : Message{ID::AFSKTxConfigure},
          samples_per_bit(samples_per_bit),
          phase_inc_mark(phase_inc_mark),
          phase_inc_space(phase_inc_space),
          repeat(repeat),
          fm_delta(fm_delta),
          symbol_count(symbol_count) {
    }

    const uint32_t samples_per_bit;
    const uint32_t phase_inc_mark;
    const uint32_t phase_inc_space;
    const uint8_t repeat;
    const uint32_t fm_delta;
    const uint8_t symbol_count;
};

class OOKConfigureMessage : public Message {
   public:
    constexpr OOKConfigureMessage(
        const uint32_t stream_length,
        const uint32_t samples_per_bit,
        const uint8_t repeat,
        const uint32_t pause_symbols,
        const uint8_t de_bruijn_length)
        : Message{ID::OOKConfigure},
          stream_length(stream_length),
          samples_per_bit(samples_per_bit),
          repeat(repeat),
          pause_symbols(pause_symbols),
          de_bruijn_length(de_bruijn_length) {
    }

    const uint32_t stream_length;
    const uint32_t samples_per_bit;
    const uint8_t repeat;
    const uint32_t pause_symbols;
    const uint8_t de_bruijn_length;
};

class SSTVConfigureMessage : public Message {
   public:
    constexpr SSTVConfigureMessage(
        const uint8_t vis_code,
        const uint32_t pixel_duration,
        const uint8_t modulation)
        : Message{ID::SSTVConfigure},
          vis_code(vis_code),
          pixel_duration(pixel_duration),
          modulation(modulation) {
    }

    const uint8_t vis_code;
    const uint32_t pixel_duration;
    const uint8_t modulation;  // sstv::sstv_modulation
};

class SSTVRXConfigureMessage : public Message {
   public:
    constexpr SSTVRXConfigureMessage(
        const uint8_t code,
        const uint8_t modulation)
        : Message{ID::SSTVRXConfigure},
          code(code),
          modulation(modulation) {
    }

    const uint8_t code;
    const uint8_t modulation;  // sstv::sstv_modulation
};

class SSTVRXProgressMessage : public Message {
   public:
    constexpr SSTVRXProgressMessage(
        const uint16_t line,
        const uint16_t total_lines)
        : Message{ID::SSTVRXProgress},
          line(line),
          total_lines(total_lines) {
    }

    const uint16_t line;
    const uint16_t total_lines;
};

class SSTVRXPhaseSlantMessage : public Message {
   public:
    constexpr SSTVRXPhaseSlantMessage(
        const int16_t phase,
        const int16_t slant)
        : Message{ID::SSTVRXPhaseSlant},
          phase(phase),
          slant(slant) {
    }

    const int16_t phase;
    const int16_t slant;
};

class SSTVRXCalibrationMessage : public Message {
   public:
    constexpr SSTVRXCalibrationMessage(
        const int16_t suggested_phase,
        const int16_t suggested_slant,
        const uint16_t sync_count)
        : Message{ID::SSTVRXCalibration},
          suggested_phase(suggested_phase),
          suggested_slant(suggested_slant),
          sync_count(sync_count) {
    }

    const int16_t suggested_phase;  // Suggested phase correction in pixels
    const int16_t suggested_slant;  // Suggested slant correction in 0.1% units
    const uint16_t sync_count;      // Number of syncs analyzed
};

class FSKConfigureMessage : public Message {
   public:
    constexpr FSKConfigureMessage(
        const uint32_t stream_length,
        const uint32_t samples_per_bit,
        const uint32_t shift,
        const uint32_t progress_notice)
        : Message{ID::FSKConfigure},
          stream_length(stream_length),
          samples_per_bit(samples_per_bit),
          shift(shift),
          progress_notice(progress_notice) {
    }

    const uint32_t stream_length;
    const uint32_t samples_per_bit;
    const uint32_t shift;
    const uint32_t progress_notice;
};

class FSKRxConfigureMessage : public Message {
   public:
    constexpr FSKRxConfigureMessage(
        const uint8_t samplesPerSymbol,
        const uint32_t syncWord,
        const uint8_t syncWordLength,
        const uint32_t preamble,
        const uint8_t preambleLength,
        const uint16_t numDataBytes)
        : Message{ID::FSKRxConfigure},
          samplesPerSymbol(samplesPerSymbol),
          syncWord(syncWord),
          syncWordLength(syncWordLength),
          preamble(preamble),
          preambleLength(preambleLength),
          numDataBytes(numDataBytes) {
    }

    const uint8_t samplesPerSymbol;
    const uint32_t syncWord;
    const uint8_t syncWordLength;
    const uint32_t preamble;
    const uint8_t preambleLength;
    const uint16_t numDataBytes;
};

class POCSAGConfigureMessage : public Message {
   public:
    constexpr POCSAGConfigureMessage(int8_t baud_config = -1)
        : Message{ID::POCSAGConfigure}, baud_config(baud_config) {
    }
    int8_t baud_config;  //-1 auto, 0=512,1=1200,2=2400
};

class APRSPacketMessage : public Message {
   public:
    constexpr APRSPacketMessage(
        const aprs::APRSPacket& packet)
        : Message{ID::APRSPacket},
          packet{packet} {
    }

    aprs::APRSPacket packet;
};

class ADSBConfigureMessage : public Message {
   public:
    constexpr ADSBConfigureMessage()
        : Message{ID::ADSBConfigure} {
    }
};

class JammerConfigureMessage : public Message {
   public:
    constexpr JammerConfigureMessage(
        const bool run,
        const jammer::JammerType type,
        const uint32_t speed)
        : Message{ID::JammerConfigure},
          run(run),
          type(type),
          speed(speed) {
    }

    const bool run;
    const jammer::JammerType type;
    const uint32_t speed;
};

class DTMFTXConfigMessage : public Message {
   public:
    constexpr DTMFTXConfigMessage(
        const uint32_t bw,
        const uint32_t tone_length,
        const uint32_t pause_length)
        : Message{ID::DTMFTXConfig},
          bw(bw),
          tone_length(tone_length),
          pause_length(pause_length) {
    }

    const uint32_t bw;
    const uint32_t tone_length;
    const uint32_t pause_length;
};

// TODO: use streaming buffer instead
// TODO: rename (not only used for requests)
class RequestSignalMessage : public Message {
   public:
    enum class Signal : char {
        FillRequest = 1,
        RogerBeepRequest = 2,
        RSSIBeepRequest = 3,
        BeepStopRequest = 4,
        Squelched = 5,
    };

    constexpr RequestSignalMessage(
        Signal signal)
        : Message{ID::RequestSignal},
          signal(signal) {
    }

    Signal signal;
};

class FIFODataMessage : public Message {
   public:
    constexpr FIFODataMessage(
        const int8_t* data)
        : Message{ID::FIFOData},
          data(data) {
    }

    const int8_t* data;
};

class CaptureThreadDoneMessage : public Message {
   public:
    constexpr CaptureThreadDoneMessage(
        uint32_t error = 0)
        : Message{ID::CaptureThreadDone},
          error{error} {
    }

    uint32_t error;
};

class ReplayThreadDoneMessage : public Message {
   public:
    constexpr ReplayThreadDoneMessage(
        uint32_t return_code = 0)
        : Message{ID::ReplayThreadDone},
          return_code{return_code} {
    }

    uint32_t return_code;
};

class SpectrumPainterBufferConfigureRequestMessage : public Message {
   public:
    constexpr SpectrumPainterBufferConfigureRequestMessage(
        uint16_t width,
        uint16_t height,
        bool update,
        int32_t bw)
        : Message{ID::SpectrumPainterBufferRequestConfigure},
          width{width},
          height{height},
          update{update},
          bw{bw} {
    }

    uint16_t width;
    uint16_t height;
    bool update;
    int32_t bw;
};

using SpectrumPainterFIFO = FIFO<std::vector<uint8_t>>;
class SpectrumPainterBufferConfigureResponseMessage : public Message {
   public:
    static constexpr size_t fifo_k = 2;

    constexpr SpectrumPainterBufferConfigureResponseMessage(
        SpectrumPainterFIFO* fifo)
        : Message{ID::SpectrumPainterBufferResponseConfigure},
          fifo{fifo} {
    }

    SpectrumPainterFIFO* fifo{nullptr};
};

class SubGhzFPRxConfigureMessage : public Message {
   public:
    constexpr SubGhzFPRxConfigureMessage(uint8_t modulation = 0, uint32_t sampling_rate = 0)
        : Message{ID::SubGhzFPRxConfigure}, modulation{modulation}, sampling_rate{sampling_rate} {
    }
    uint8_t modulation = 0;  // 0 am, 1 fm
    uint32_t sampling_rate = 0;
};

class WeatherDataMessage : public Message {
   public:
    constexpr WeatherDataMessage(
        uint8_t sensorType = 0,
        uint64_t decode_data = 0xFFFFFFFF)
        : Message{ID::WeatherData},
          sensorType{sensorType},
          decode_data{decode_data} {
    }
    uint8_t sensorType = 0;
    uint64_t decode_data = 0;
    uint8_t payload_len = 0;
    uint8_t payload[32]{};  // raw frame for protocols longer than 64 bits (WS_MAX_PAYLOAD)
};

class SubGhzDDataMessage : public Message {
   public:
    constexpr SubGhzDDataMessage(
        uint8_t sensorType = 0,
        uint16_t bits = 0,
        uint64_t data = 0)
        : Message{ID::SubGhzDData},
          sensorType{sensorType},
          bits{bits},
          data{data} {
    }
    uint8_t sensorType = 0;
    uint16_t bits = 0;
    uint64_t data = 0;
};

class GPSPosDataMessage : public Message {
   public:
    constexpr GPSPosDataMessage(
        float lat = 200.0,
        float lon = 200.0,
        int32_t altitude = 0,
        int32_t speed = 0,
        uint8_t satinuse = 0)
        : Message{ID::GPSPosData},
          lat{lat},
          lon{lon},
          altitude{altitude},
          speed{speed},
          satinuse{satinuse} {
    }
    float lat = 200.0;
    float lon = 200.0;
    int32_t altitude = 0;
    int32_t speed = 0;
    uint8_t satinuse = 0;
};

class OrientationDataMessage : public Message {
   public:
    constexpr OrientationDataMessage(
        uint16_t angle = 400,
        int16_t tilt = 400)
        : Message{ID::OrientationData},
          angle{angle},
          tilt{tilt} {
    }
    uint16_t angle = 400;  //>360 -> no orientation set
    int16_t tilt = 400;
};

class EnvironmentDataMessage : public Message {
   public:
    constexpr EnvironmentDataMessage(
        float temperature = 0,
        float humidity = 0,
        float pressure = 0)
        : Message{ID::EnvironmentData},
          temperature{temperature},
          humidity{humidity},
          pressure{pressure} {
    }
    float temperature = 0;  // celsius
    float humidity = 0;     // percent (rh)
    float pressure = 0;     // hpa
};

class LightDataMessage : public Message {
   public:
    constexpr LightDataMessage(

        uint16_t light = 0)
        : Message{ID::LightData},
          light{light} {
    }
    uint16_t light = 0;  // lux
};

class AudioBeepMessage : public Message {
   public:
    constexpr AudioBeepMessage(
        uint32_t freq = 1000,
        uint32_t sample_rate = 24000,
        uint32_t duration_ms = 100)
        : Message{ID::AudioBeep},
          freq{freq},
          sample_rate{sample_rate},
          duration_ms{duration_ms} {
    }
    uint32_t freq = 1000;
    uint32_t sample_rate = 24000;
    uint32_t duration_ms = 100;
};

class PocsagTosendMessage : public Message {
   public:
    constexpr PocsagTosendMessage(
        uint16_t baud = 1200,
        uint8_t type = 2,
        char function = 'A',
        char polarity = 'S', /* 'S' = Standard (CCIR Rec. 584), 'I' = Inverted */
        uint8_t msglen = 0,
        uint8_t msg[31] = {0},
        uint64_t addr = 0)
        : Message{ID::PocsagTosend},
          baud{baud},
          type{type},
          function{function},
          polarity{polarity},
          msglen{msglen},
          addr{addr} {
        memcpy(this->msg, msg, 31);
    }
    uint16_t baud = 1200;
    uint8_t type = 2;
    char function = 'A';
    char polarity = 'S'; /* 'S' = Standard, 'I' = Inverted */
    uint8_t msglen = 0;
    uint8_t msg[31] = {0};
    uint64_t addr = 0;
};

class BatteryStateMessage : public Message {
   public:
    constexpr BatteryStateMessage(
        uint8_t valid_mask,
        uint8_t percent,
        bool on_charger,
        uint16_t voltage,
        bool battMayChanged)
        : Message{ID::BatteryStateData},
          valid_mask{valid_mask},
          percent{percent},
          on_charger{on_charger},
          voltage{voltage},
          battMayChanged{battMayChanged} {
    }
    uint8_t valid_mask = 0;
    uint8_t percent = 0;
    bool on_charger = false;
    uint16_t voltage = 0;  // mV
    bool battMayChanged = false;
};

class ProtoViewDataMessage : public Message {
   public:
    constexpr ProtoViewDataMessage()
        : Message{ID::ProtoViewData} {}
    int32_t times[100] = {0};  // positive: high, negative: low
    uint16_t timeptr = 0;
    const uint16_t maxptr = 99;
};

class FreqChangeCommandMessage : public Message {
   public:
    constexpr FreqChangeCommandMessage(int64_t freq)
        : Message{ID::FreqChangeCommand}, freq{freq} {}
    int64_t freq = 0;
};

class I2CDevListChangedMessage : public Message {
   public:
    constexpr I2CDevListChangedMessage()
        : Message{ID::I2CDevListChanged} {}
};

class WeFaxRxConfigureMessage : public Message {
   public:
    constexpr WeFaxRxConfigureMessage(uint8_t lpm, uint8_t ioc)
        : Message{ID::WeFaxRxConfigure},
          lpm{lpm},
          ioc{ioc} {
    }
    uint8_t lpm = 120;
    uint8_t ioc = 0;
};

class WeFaxRxStatusDataMessage : public Message {
   public:
    constexpr WeFaxRxStatusDataMessage(uint8_t state)
        : Message{ID::WeFaxRxStatusData},
          state{state} {
    }
    uint8_t state = 0;
};

class WeFaxRxImageDataMessage : public Message {
   public:
    constexpr WeFaxRxImageDataMessage()
        : Message{ID::WeFaxRxImageData} {}
    uint8_t image[400]{0};
    uint32_t cnt = 0;
};

class NoaaAptRxConfigureMessage : public Message {
   public:
    constexpr NoaaAptRxConfigureMessage()
        : Message{ID::NoaaAptRxConfigure} {}
};

class NoaaAptRxStatusDataMessage : public Message {
   public:
    constexpr NoaaAptRxStatusDataMessage(uint8_t state)
        : Message{ID::NoaaAptRxStatusData},
          state{state} {
    }
    uint8_t state = 0;
};

class NoaaAptRxImageDataMessage : public Message {
   public:
    constexpr NoaaAptRxImageDataMessage()
        : Message{ID::NoaaAptRxImageData} {}
    uint8_t image[400]{0};
    uint32_t cnt = 0;
};

class FlexPacketMessage : public Message {
   public:
    constexpr FlexPacketMessage(const flex::FlexPacket& packet)
        : Message{ID::FlexPacket},
          packet{packet} {
    }
    flex::FlexPacket packet;
};

class FlexStatsMessage : public Message {
   public:
    constexpr FlexStatsMessage(const flex::FlexStats& stats)
        : Message{ID::FlexStats},
          stats{stats} {
    }
    flex::FlexStats stats;
};

class FlexConfigureMessage : public Message {
   public:
    constexpr FlexConfigureMessage()
        : Message{ID::FlexConfigure} {
    }
};

class FlexDebugMessage : public Message {
   public:
    constexpr FlexDebugMessage(const uint32_t val1, const uint32_t val2, const char* msg)
        : Message{ID::FlexDebug},
          val1{val1},
          val2{val2},
          text{} {
        size_t i = 0;
        while (i < sizeof(text) - 1 && msg[i] != '\0') {
            text[i] = msg[i];
            i++;
        }
        text[i] = '\0';
    }

    uint32_t val1;
    uint32_t val2;
    char text[64];
};

class SubCarDataMessage : public Message {
   public:
    constexpr SubCarDataMessage(
        uint8_t sensorType = 0,
        uint16_t bits = 0,
        uint64_t data = 0,
        uint64_t data2 = 0)
        : Message{ID::SubCarData},
          sensorType{sensorType},
          bits{bits},
          data{data},
          data2{data2} {
    }
    uint8_t sensorType = 0;
    uint16_t bits = 0;
    uint64_t data = 0;
    uint64_t data2 = 0;
};

class MorseRXDataMessage : public Message {
   public:
    constexpr MorseRXDataMessage()
        : Message{ID::MorseRXData} {}
    int32_t state_durations[5] = {0};  // positive: high, negative: low
    uint8_t state_cnt = 0;
    bool clipped = false;
    const uint8_t maxptr = 4;
};

class MorseRXfreqMessage : public Message {
   public:
    constexpr MorseRXfreqMessage()
        : Message{ID::MorseRXfreq} {}
    uint32_t measured_frequency = 0;
};

class MorseRXConfigureMessage : public Message {
   public:
    constexpr MorseRXConfigureMessage(uint8_t mode)
        : Message{ID::MorseRXConfig},
          mode{mode} {}
    uint8_t mode = 0;
};

class TXDisabledMessage : public Message {
   public:
    constexpr TXDisabledMessage()
        : Message{ID::TXDisabled} {
    }
};

class MorseTXConfigureMessage : public Message {
   public:
    constexpr MorseTXConfigureMessage(uint8_t modulation, uint32_t tone, float fm_delta)
        : Message{ID::MorseTXConfigure},
          modulation{modulation},
          tone{tone},
          fm_delta{fm_delta} {}

    uint8_t modulation = 0;
    uint32_t tone = 0;
    float fm_delta = 0;
};

class MorseTXkeyMessage : public Message {
   public:
    constexpr MorseTXkeyMessage(bool key_down)
        : Message{ID::MorseTXkey},
          key_down{key_down} {}
    bool key_down = false;
};

class StreamTXConfigurationMessage : public Message {
   public:
    constexpr StreamTXConfigurationMessage(uint32_t deviation, uint8_t mode)
        : Message{ID::StreamTXConfiguration},
          deviation{deviation},
          mode{mode} {}

    uint32_t deviation = 60000;  // used in 2fsk
    uint8_t mode = 0;            // am = 0, 2fsk = 1
};

class RTTYDataMessage : public Message {
   public:
    constexpr RTTYDataMessage(uint16_t baud = 4545, uint16_t shift = 170, int16_t mark_tone = -85, int16_t space_tone = 85, uint8_t stopbits = 3, bool inverted = false)
        : Message{ID::RTTYData},
          baud(baud),
          shift(shift),
          mark_tone(mark_tone),
          space_tone(space_tone),
          inverted(inverted),
          stopbits(stopbits) {
    }
    uint8_t data[490]{0};      // 5bit data, stored on 8 bits.
    uint16_t data_len = 0;     // count of data sent
    uint16_t baud = 4545;      // /100 baud 45.45 = 4545
    uint16_t shift = 170;      // hz
    int16_t mark_tone = 0;     // for tx., hz
    int16_t space_tone = 170;  // hz
    bool inverted = false;     // for tx, if true, mark and space tones are swapped.
    uint8_t stopbits = 3;      // doubled value is stored here, so stop 2 = 1 stop bit, 3 = 1.5, 4 = 2.
    static constexpr uint16_t max_len = 490;
};

class NotificationDataMessage : public Message {
   public:
    constexpr NotificationDataMessage(const char* source_app, const char* title, const char* message, uint8_t icon = 0, uint16_t timeout = 10000)
        : Message{ID::NotificationData},
          icon(icon),
          timeout(timeout) {
        if (source_app) {
            size_t len = std::min(strlen(source_app), (size_t)19);
            memcpy(this->source_app, source_app, len);
            this->source_app[len] = '\0';
        }
        if (title) {
            size_t len = std::min(strlen(title), (size_t)49);
            memcpy(this->title, title, len);
            this->title[len] = '\0';
        }
        if (message) {
            size_t len = std::min(strlen(message), (size_t)299);
            memcpy(this->message, message, len);
            this->message[len] = '\0';
        }
    }
    char source_app[20]{0};  // source application name, null-terminated, max 19 chars + null
    char title[50]{0};       // title, null-terminated, max 49 chars + null
    char message[300]{0};    // message, null-terminated, max 299 chars + null
    uint8_t icon = 0;
    uint16_t timeout = 10000;
};

// Sent M0→M4: a tone detection event (tone ended, or periodic update while active)
class ToneDetectDataMessage : public Message {
   public:
    constexpr ToneDetectDataMessage()
        : Message{ID::ToneDetectData} {}
    uint32_t freq_hz{0};      // Dominant audio frequency in Hz (0 = silence)
    uint32_t duration_ms{0};  // How long the tone lasted in milliseconds
    bool tone_end{false};     // True = tone just ended; False = tone still active (periodic update)
};

// Sent M4→M0: configure the tone detector
class ToneDetectConfigureMessage : public Message {
   public:
    constexpr ToneDetectConfigureMessage(uint8_t squelch = 0, uint32_t ctcss_freq_x10 = 0)
        : Message{ID::ToneDetectConfig},
          squelch_level{squelch},
          ctcss_freq_x10{ctcss_freq_x10} {}
    uint8_t squelch_level{0};
    uint32_t ctcss_freq_x10{0};  // CTCSS frequency × 10 (e.g. 1000 = 100.0 Hz); 0 = None
};

class VorRxConfigureMessage : public Message {
   public:
    constexpr VorRxConfigureMessage(bool enabled = true)
        : Message{ID::VorRxConfigure},
          enabled{enabled} {
    }

    bool enabled{true};
};

class VorRxStatusDataMessage : public Message {
   public:
    constexpr VorRxStatusDataMessage(
        uint16_t phase_deg = 0,
        uint16_t radial_deg = 0,
        uint16_t reference_level = 0,
        uint16_t variable_level = 0,
        uint8_t quality = 0,
        bool valid = false,
        bool to_from = false)
        : Message{ID::VorRxStatusData},
          phase_deg{phase_deg},
          radial_deg{radial_deg},
          reference_level{reference_level},
          variable_level{variable_level},
          quality{quality},
          valid{valid},
          to_from{to_from} {
    }

    uint16_t phase_deg{0};
    uint16_t radial_deg{0};
    uint16_t reference_level{0};
    uint16_t variable_level{0};
    uint8_t quality{0};
    bool valid{false};
    bool to_from{false};
};

class VorTxConfigureMessage : public Message {
   public:
    VorTxConfigureMessage(
        uint16_t radial_deg = 0,
        bool ident_enabled = true,
        const char* ident = "",
        bool enabled = true)
        : Message{ID::VorTxConfigure},
          radial_deg{radial_deg},
          ident_enabled{ident_enabled},
          enabled{enabled} {
        size_t i = 0;
        for (; ident && ident[i] && i < sizeof(ident_text) - 1; ++i)
            ident_text[i] = ident[i];
        ident_text[i] = '\0';
    }

    uint16_t radial_deg{0};
    bool ident_enabled{true};
    bool enabled{true};
    char ident_text[8]{};  // CW identifier, null-terminated (max 7 chars)
};

class FlexTosendMessage : public Message {
   public:
    constexpr FlexTosendMessage(
        uint64_t capcode = 0,
        uint8_t type = 0,
        uint8_t msglen = 0,
        const uint8_t* msg = nullptr)
        : Message{ID::FlexTosend},
          capcode{capcode},
          type{type},
          msglen{msglen} {
        if (msg)
            memcpy(this->msg, msg, 240);
    }
    uint64_t capcode = 0;
    uint8_t type = 0;
    uint8_t msglen = 0;
    uint8_t msg[240] = {0};
};

class HunterConfigMessage : public Message {
   public:
    uint32_t energy_threshold{5000};
    uint32_t hangtime_ms{500};
    bool start{false};
    constexpr HunterConfigMessage()
        : Message{ID::HunterConfig} {}
};

class HunterTriggerMessage : public Message {
   public:
    uint32_t energy{0};
    constexpr HunterTriggerMessage()
        : Message{ID::HunterTrigger} {}
};

class HunterStopMessage : public Message {
   public:
    constexpr HunterStopMessage()
        : Message{ID::HunterStop} {}
};

struct RDSGroupMessage : public Message {
    constexpr RDSGroupMessage(
        uint16_t a,
        uint16_t b,
        uint16_t c,
        uint16_t d,
        bool c_prime,
        bool is_debug,
        uint32_t dbg_1,
        uint32_t dbg_2)
        : Message{ID::RdsData}, block_a{a}, block_b{b}, block_c{c}, block_d{d}, is_c_prime{c_prime}, is_debug{is_debug}, debug_1{dbg_1}, debug_2{dbg_2} {}

    uint16_t block_a;
    uint16_t block_b;
    uint16_t block_c;
    uint16_t block_d;
    bool is_c_prime;

    bool is_debug;
    uint32_t debug_1;
    uint32_t debug_2;
};

// One AVLC frame from VHF Data Link mode 2 (vdl2_rx), FCS checked and taken off.
struct VDL2FrameMessage : public Message {
    static constexpr size_t MAX_LENGTH = 320;

    constexpr VDL2FrameMessage()
        : Message{ID::VDL2Frame} {}

    uint16_t length{0};    // as received; the data holds at most MAX_LENGTH of it
    int16_t offset_hz{0};  // carrier offset seen at the sync
    uint8_t data[MAX_LENGTH]{};
};

// One CC1101 packet for the Sub-GHz chat (subghz_chat), CRC checked.
struct SubGhzChatPacketMessage : public Message {
    static constexpr size_t MAX_LENGTH = 60;
    // receiver noise for the app's encryption, kept up by the image at the start of bb_data
    static constexpr size_t NOISE_POOL_BYTES = 32;

    constexpr SubGhzChatPacketMessage()
        : Message{ID::SubGhzChatPacket} {}

    uint8_t length{0};
    uint8_t data[MAX_LENGTH]{};
};

// Signature Detect (external app signature_detect, baseband PSDT). M0 -> M4.
struct SignatureDetectConfigMessage : public Message {
    static constexpr uint8_t MODE_IDLE = 0;
    static constexpr uint8_t MODE_GFSK_BLE = 1;
    static constexpr uint8_t MODE_SWEEP = 2;         // reserved: sweeps are entered with SpecAnConfig
    static constexpr uint8_t MODE_BURST = 3;         // reserved
    static constexpr uint8_t MODE_GFSK_PATTERN = 4;  // 4 Msps GFSK front end, sync word search (sync_* below)
    static constexpr uint8_t MODE_DSSS = 5;          // 4 Msps 802.11b 1 Mb/s receiver on a slice of a Wi-Fi channel (Wi-Fi Remote ID)
    static constexpr uint8_t MODE_FSK = 8;           // 4 Msps narrowband FSK frame receiver (sigdet_fsk_rx.hpp) with the rules of *fsk (Z-Wave R1 is one), the M0 tunes target = the rules' f; channel: the app's tag
    static constexpr uint8_t MODE_METER = 7;         // 4 Msps track meter (fox hunt): gap-free power at the target, channel = log2 of the boxcar (2, 4, 6, 8)
    static constexpr uint8_t MODE_DRONEID = 6;       // 15.36 Msps DJI DroneID burst detector (presence only), LO on the DroneID centre (HackRF Pro only)
    static constexpr uint8_t FLAG_PASS_ALL = 0x01;   // BLE: forward every CRC valid advert, not only Remote ID (never in MODE_DSSS)
    static constexpr uint8_t FLAG_RESET_STATS = 0x02;
    static constexpr uint8_t FLAG_CONJUGATE = 0x04;    // bring-up aid: mirror the spectrum (I/Q swapped)
    static constexpr uint8_t FLAG_WIFI_LEGACY = 0x08;  // MODE_DSSS bench aid: also forward French / legacy DJI drone IEs (raw)
    static constexpr uint8_t FLAG_TARGET = 0x80;       // MODE_GFSK_BLE: forward every advert of AdvA = sync_hi:sync_lo (low 48 bits, MSB first) and nothing else (track view)
    static constexpr uint8_t FLAG_FWD_ADV = 0x40;      // MODE_GFSK_BLE: also forward other CRC valid adverts with AdvData (SOURCE_BLE_OTHER; deduplicated per address and payload for 1 s, rate capped)
    static constexpr uint8_t SEARCH_SHIFT = 4;         // MODE_DSSS flags bits 4-5: search phases - 1 (1..3 = 2..4), 0 = the image's default (bring-up)
    static constexpr uint8_t SEARCH_MASK = 0x30;
    static constexpr uint8_t PAT_INVERT = 0x01;   // pat_flags: bit 1 = negative deviation
    static constexpr uint8_t PAT_RATE_SHIFT = 1;  // pat_flags bits 1-2: sigdet::PatternRate (only 0, 1 Mb/s, is built)

    constexpr SignatureDetectConfigMessage(
        uint32_t sampling_rate,
        uint8_t mode,
        uint8_t channel,
        uint8_t flags)
        : Message{ID::SignatureDetectConfig},
          sampling_rate{sampling_rate},
          mode{mode},
          channel{channel},
          flags{flags} {}

    uint32_t sampling_rate;
    uint8_t mode;
    uint8_t channel;  // BLE channel index, 37..39; MODE_GFSK_PATTERN: the app's tag, copied into the hits; MODE_DSSS: Wi-Fi channel number; MODE_DRONEID: the app's tag, copied into the detections
    uint8_t flags;
    int8_t slice_mhz{0};  // MODE_DSSS: slice centre - channel centre, -2 or +2 (the M0 tunes target = channel + slice + 1 MHz)
    /* MODE_GFSK_PATTERN only: sync word right aligned (last bit in bit 0 of sync_lo), 8..64 bits,
     * bit errors allowed (the M4 clamps it to ceil(sync_bits / 16) - 1), payload bits returned. */
    uint32_t sync_hi{0};
    uint32_t sync_lo{0};
    uint8_t sync_bits{0};
    uint8_t max_errors{0};
    uint8_t payload_bits{0};
    uint8_t pat_flags{0};
    /* MODE_FSK only: the dwell's front end and rules, copied by the M4 while it handles this message
     * (the M0 waits for it): no lifetime beyond the call. */
    const sigdet::FskDwell* fsk{nullptr};
};

// One Remote ID frame (or, with FLAG_PASS_ALL, any CRC valid BLE advert), by value. M4 -> M0.
// Other sources (an ESP32 WiFi RID feed later) send the same message with adv[] made up as
// 1E 16 FA FF 0D counter message(25), so the app has one parser for all of them.
struct SignatureDetectRidMessage : public Message {
    static constexpr uint8_t SOURCE_BLE = 0;
    static constexpr uint8_t SOURCE_WIFI_BEACON = 1;
    static constexpr uint8_t SOURCE_WIFI_NAN = 2;
    static constexpr uint8_t SOURCE_ESP32_BLE = 3;
    static constexpr uint8_t SOURCE_BLE_VENDOR = 4;  // on-board BLE, not Remote ID: a drone maker's advert (sigdet_ble_vendor.hpp)
    static constexpr uint8_t SOURCE_BLE_OTHER = 5;   // on-board BLE, any other advert (FLAG_FWD_ADV: signature rules, survey log)
    static constexpr uint8_t SOURCE_SHELL = 7;
    static constexpr size_t ADV_MAX = 31;

    constexpr SignatureDetectRidMessage()
        : Message{ID::SignatureDetectRid} {}

    uint32_t t_ms{0};     // baseband time of the packet's end
    int16_t rssi_cdb{0};  // in-channel power, centi-dB re full scale of the int8 samples
    int16_t cfo_khz{0};   // carrier offset from the channel centre
    uint8_t chan{0};      // BLE channel index
    uint8_t pdu_type{0};  // PDU header byte 0 (type in the low nibble, TxAdd in bit 6)
    uint8_t source{SOURCE_BLE};
    uint8_t addr[6]{};       // AdvA, most significant byte first
    uint8_t adv_len{0};      // bytes in adv[]
    uint8_t adv[ADV_MAX]{};  // AdvData
};

// Counters of the Signature Detect baseband, every 500 ms. M4 -> M0.
struct SignatureDetectStatsMessage : public Message {
    constexpr SignatureDetectStatsMessage()
        : Message{ID::SignatureDetectStats} {}

    /* The M4 RAM that PSDT lends the app (scratch below): the first kScratchRf
     * bytes hold the RF detector's arrays and the spec_an::Sweep, the next
     * kScratchMap a map view (GeoMapView, ~10 KB) while one is open, the last
     * kScratchSig the signature class rules and the survey log's tables
     * (sigdet_sig.hpp, sigdet_survey.hpp), so that none of them needs the M0 heap. */
    static constexpr uint16_t kScratchRf = 4608;
    static constexpr uint16_t kScratchMap = 10752;
    static constexpr uint16_t kScratchSig = 7168;
    static constexpr uint16_t kScratchSize = kScratchRf + kScratchMap + kScratchSig;

    uint32_t aa_hits{0};  // access address syncs that started a packet
    uint32_t crc_bad{0};  // ... that failed the CRC
    uint32_t ok{0};       // CRC valid adverts
    uint32_t rid_ok{0};   // ... of them carrying Open Drone ID
    uint16_t drops{0};    // DMA buffers the M4 missed (all causes)
    uint16_t cpu_pct{0};  // share of the M4 spent in execute() over the period
    uint8_t chan{0};
    uint8_t mode{0};
    /* Diagnostics. Counters are since the last FLAG_RESET_STATS, max_* since the last stats message. */
    uint32_t triggers{0};        // access address pattern matches that ran the soft check
    uint32_t len_bad{0};         // syncs that failed the length check
    uint16_t max_exec_us{0};     // longest execute()
    uint16_t max_lag_us{0};      // latest end of an execute() after its buffer was complete (>= 512 at 4 Msps: one lost)
    uint16_t max_start_us{0};    // latest start of an execute() after its buffer was complete (wake-up latency)
    uint16_t slow_execs{0};      // execute() calls over 400 us
    uint16_t hop_skips{0};       // buffers discarded while a retune settles
    uint16_t drops_hop{0};       // drops within 16 buffers of a channel change
    uint16_t drops_late{0};      // drops right after an execute() that ended past its deadline
    uint16_t q_fail{0};          // messages to the M0 lost to a full application queue
    uint16_t sync_capped{0};     // triggers ignored at the per-buffer soft check cap
    uint16_t dup_suppressed{0};  // Remote ID frames not forwarded: same advert within 300 ms
    uint16_t rate_capped{0};     // frames not forwarded: over 8 per 100 ms
    uint16_t scratch_size{0};    // bytes at scratch: M4 RAM the app may use as working memory (never touched by the M4)
    uint8_t* scratch{nullptr};   // valid while the PSDT image runs
    /* Drops by phase (the sum is `drops`): demodulating BLE, sweeping, or within 8 buffers of
     * a phase change (sample rate switch). */
    uint16_t drops_ble{0};
    uint16_t drops_sweep{0};
    uint16_t drops_switch{0};
    /* Lost pushes by message (q_fail is their sum), and by cause: the event thread held the
     * queue's lock (the rest: queue full). */
    uint16_t qf_rid{0};
    uint16_t qf_stats{0};
    uint16_t qf_lock{0};
    uint16_t q_peak{0};   // most bytes queued at a push since the last stats message
    uint32_t rid_fwd{0};  // Remote ID frames (or pass-all adverts) queued to the M0
    /* Pattern mode (MODE_GFSK_PATTERN) and DJI adverts, since the last FLAG_RESET_STATS. */
    uint32_t pat_cands{0};    // clean 16 bit pieces: full distance counted
    uint32_t pat_matches{0};  // ... within the allowed errors (all phases)
    uint32_t pat_hits{0};     // merged hits (one per packet)
    uint32_t pat_fwd{0};      // ... queued to the M0
    uint16_t pat_capped{0};   // matches lost to the per-block candidate list
    uint16_t pat_rcap{0};     // hits not forwarded: over 8 per 100 ms
    uint16_t drops_pat{0};    // drops while in pattern mode (part of `drops`)
    uint16_t vendor_dup{0};   // DJI adverts not forwarded: same advert within 1 s
    uint32_t vendor_ok{0};    // CRC valid adverts matching the DJI table (not Remote ID)
    uint32_t vendor_fwd{0};   // ... queued to the M0
    /* Wi-Fi Remote ID (MODE_DSSS), counters only (no frame content), since the last FLAG_RESET_STATS. */
    uint32_t wifi_triggers{0};     // SYNC detections
    uint32_t wifi_frames{0};       // SFDs found: 1 Mb/s long preamble frames seen
    uint32_t wifi_hdr_bad{0};      // ... PLCP header CRC failed
    uint32_t wifi_filtered{0};     // ... not a beacon / probe response / action frame (dropped at byte 0)
    uint32_t wifi_fcs_bad{0};      // ... FCS failed
    uint32_t wifi_fcs_ok{0};       // ... FCS valid
    uint32_t wifi_beacon{0};       // FCS valid by type: beacons
    uint32_t wifi_probe{0};        // ... probe responses
    uint32_t wifi_action{0};       // ... action frames
    uint32_t wifi_rid{0};          // ... carrying Remote ID (ODID vendor IE or NAN)
    uint32_t wifi_fwd{0};          // Remote ID frames queued to the M0
    uint16_t wifi_sync_lost{0};    // triggers without SFD
    uint16_t wifi_hdr_2m{0};       // PLCP headers of 2 Mb/s frames (not demodulated)
    uint16_t wifi_legacy{0};       // French / legacy DJI IE beacons (forwarded with FLAG_WIFI_LEGACY only)
    uint16_t wifi_dup{0};          // Remote ID frames not forwarded: same frame within 300 ms
    uint16_t wifi_rcap{0};         // ... over the rate cap
    uint16_t wifi_busy{0};         // FCS valid frames dropped: both hand-off slots to the event thread busy
    uint16_t wifi_qf{0};           // Remote ID frames lost to a full (or locked) application queue
    uint16_t wifi_overflows{0};    // input ring overflows (backlog dropped)
    uint16_t wifi_backlog{0};      // most samples waiting in the input ring (since the last stats message)
    uint16_t wifi_max_step_us{0};  // longest demodulator step (since the last stats message)
    uint16_t drops_wifi{0};        // drops while in MODE_DSSS (part of `drops`)
    /* Measured cost of the DSSS demodulator (DWT cycles, mean over the last
     * stats period, 0 if none): per group of 4 samples in each state, per
     * CFO pass; and the search mode in use (2, 3, 4). */
    uint16_t wifi_cyc_search{0};
    uint16_t wifi_cyc_sync{0};
    uint16_t wifi_cyc_lock{0};
    uint16_t wifi_cyc_cfo{0};
    uint8_t wifi_phases{0};
    uint32_t wifi_shed{0};  // Search groups run on 2 phases to shed load (mode 3), since the last FLAG_RESET_STATS
    /* DJI DroneID detector (MODE_DRONEID), counters since the last FLAG_RESET_STATS, max / mean since the last stats message. */
    uint32_t did_cands{0};      // energy bursts of DroneID duration (535..720 us)
    uint32_t did_evals{0};      // ... captured and tested by stage B (CP fold + ZC root 600)
    uint32_t did_dets{0};       // ... passed: SignatureDetectDroneIdMessage sent (or lost: did_qf)
    uint16_t did_skips{0};      // candidates not captured: snapshot still frozen for stage B, or a buffer lost mid-burst
    uint16_t did_capped{0};     // candidates captured but not tested: over the stage B rate cap
    uint16_t did_b_max_us{0};   // longest stage B (event thread, CPU time incl. preemption by the baseband thread)
    uint16_t did_b_mean_us{0};  // mean stage B
    uint16_t drops_did{0};      // drops while in MODE_DRONEID (part of `drops`)
    uint16_t did_qf{0};         // detections lost to a full (or locked) application queue
    /* Other adverts (FLAG_FWD_ADV), since the last FLAG_RESET_STATS. */
    uint32_t other_fwd{0};  // queued to the M0
    uint16_t other_dup{0};  // not forwarded: same address and payload within 1 s
    uint16_t other_qf{0};   // lost to a full (or locked) application queue or the rate cap
    /* FSK rules (MODE_FSK, Z-Wave included), since the last FLAG_RESET_STATS. */
    uint32_t fsk_syncs{0};  // a rule's pattern found
    uint32_t fsk_ok{0};     // frames with a passing check (queued or lost: fsk_qf)
    uint16_t fsk_bad{0};    // bad length or check
    uint16_t fsk_qf{0};     // frames lost to a full application queue
    uint32_t fsk_nochk{0};  // frames of rules without a check (sync + length only)
};

// One DJI DroneID burst detection (MODE_DRONEID), by value. M4 -> M0. Presence only: a burst of the
// OcuSync DroneID shell (duration, CP structure, Zadoff-Chu root 600 reference symbol); the payload is
// not demodulated (O4 payloads are encrypted anyway).
struct SignatureDetectDroneIdMessage : public Message {
    constexpr SignatureDetectDroneIdMessage()
        : Message{ID::SignatureDetectDroneId} {}

    uint32_t t_ms{0};      // baseband time at the end of stage B (the burst ended up to ~20 ms before)
    int16_t level_cdb{0};  // mean burst power over the on-time, centi-dB re full scale (|x| = 128 int8)
    int16_t snr_cdb{0};    // on-time power over the energy floor, centi-dB
    int16_t cfo_hz{0};     // fractional carrier offset, Hz (within +-7.5 kHz; whole subcarriers are not resolved)
    uint16_t zc_milli{0};  // ZC root 600 correlation x 1000 (detect at >= 60; real bursts ~150-990)
    uint16_t cp_milli{0};  // CP coherence x 1000
    uint16_t dur_us{0};    // on-time
    uint8_t nsym{0};       // layout: 8 symbols (O1 / O4) or 9 (O2)
    uint8_t tag{0};        // SignatureDetectConfigMessage::channel of the dwell
};

// Track meter report (MODE_METER, sigdet_meter.hpp), every ~50 ms. M4 -> M0.
struct SignatureDetectMeterMessage : public Message {
    constexpr SignatureDetectMeterMessage()
        : Message{ID::SignatureDetectMeter} {}

    uint32_t t_ms{0};
    int16_t peak_cdb{-12000};   // strongest 256 us block of the interval, centi-dB re full scale (|x| = 128)
    int16_t mean_cdb{-12000};   // mean of the interval
    int16_t floor_cdb{-12000};  // pulse detector floor
    uint16_t pulses{0};         // pulse starts (+6 dB over the floor) in the interval
    uint32_t clip{0};           // raw I / Q samples at |x| >= 120 in the interval
    uint32_t period_us{0};      // median pulse start-to-start (last <= 8), 0 = unknown
    uint32_t last_period_us{0};
    uint32_t width_us{0};  // last pulse width
    uint8_t log2d{0};      // boxcar 2^log2d of 4 Msps
};

// One FSK frame (MODE_FSK): a rule's frame whose check passed (or, for a rule without a check, sync and length
// only), its length and first <= 16 bytes (Z-Wave: the header), never more. M4 -> M0.
struct SignatureDetectFskMessage : public Message {
    static constexpr uint8_t FLAG_CHECKED = 0x01;  // the rule's check passed

    constexpr SignatureDetectFskMessage()
        : Message{ID::SignatureDetectFsk} {}

    uint32_t t_ms{0};
    uint32_t rate_bps{0};      // bit rate the chip clock settled to
    int16_t rssi_cdb{-12000};  // mean power over the frame, centi-dB re full scale (|x| = 128) in the dwell's channel
    int16_t off_hz{0};         // carrier offset
    uint16_t dev_hz{0};        // mean |deviation| over the frame
    uint8_t tag{0};            // the rule (FskRule::tag, the app's index)
    uint8_t len{0};            // frame bytes after the sync, the check included (Z-Wave: the length byte)
    uint8_t nhdr{0};           // bytes in hdr
    uint8_t violations{0};     // Manchester chip pairs that were not complementary
    uint8_t flags{0};
    uint8_t hdr[16]{};
};

// One Wi-Fi Remote ID frame (MODE_DSSS), by value. M4 -> M0. Only the first wire_size() bytes are
// queued (MessageQueue::push_prefix): a typical ODID beacon is ~150 bytes of the 280.
struct SignatureDetectWifiMessage : public Message {
    constexpr SignatureDetectWifiMessage()
        : Message{ID::SignatureDetectWifi} {}

    sigdet::SignatureDetectWifiRid rid{};

    size_t wire_size() const { return sizeof(Message) + rid.wire_size(); }
};

// One pattern hit (MODE_GFSK_PATTERN), by value. M4 -> M0.
struct SignatureDetectPatternHitMessage : public Message {
    constexpr SignatureDetectPatternHitMessage()
        : Message{ID::SignatureDetectPatternHit} {}

    uint32_t t_ms{0};        // baseband time of the hit
    uint32_t payload_hi{0};  // payload bits right aligned, first bit received in the MSB of the payload_bits
    uint32_t payload_lo{0};
    int16_t rssi_cdb{0};  // in-channel power over sync and payload, centi-dB re full scale
    uint8_t tag{0};       // SignatureDetectConfigMessage::channel of the dwell
    uint8_t errors{0};    // sync bits wrong
    uint8_t phase{0};     // sample phase, 0..3
    uint8_t payload_bits{0};
};

struct TetraBurstMessage : public Message {
    constexpr TetraBurstMessage(
        const uint8_t* bits,
        bool inv,
        uint8_t err)
        : Message(Message::ID::TetraBsch),
          inverted(inv),
          sync_errors(err),
          payload{} {
        for (size_t i = 0; i < 63; i++)
            payload[i] = bits[i];
    }

    bool inverted;
    uint8_t sync_errors;

    // 500 bit
    std::array<uint8_t, 63> payload;
};

struct TetraDnbMessage : public Message {
    constexpr TetraDnbMessage(
        const uint8_t* bits,
        bool inv,
        uint8_t err,
        bool p_train)
        : Message(Message::ID::TetraDnb),
          inverted(inv),
          train_errors(err),
          is_p_train(p_train),
          payload{} {
        for (size_t i = 0; i < 54; i++)
            payload[i] = bits[i];
    }

    bool inverted;
    uint8_t train_errors;
    bool is_p_train;

    // 432 TCH type-5 bits: 216 bits before the training sequence + 216 bits after.
    std::array<uint8_t, 54> payload;
};
/* Spectrum Analyzer ******************************************************/

/* M0 -> M4, once per plan change. See spec_an_shared.hpp for the protocol. */
class SpecAnConfigMessage : public Message {
   public:
    constexpr SpecAnConfigMessage(
        spec_an::Shared* shared,
        uint32_t sampling_rate,
        uint16_t gen,
        uint8_t fft_log2n,
        uint16_t avg_blocks,
        int16_t bin_first,
        uint16_t bins_per_slice,
        uint16_t slices,
        uint64_t px_per_bin_num,
        uint64_t px_per_bin_den,
        spec_an::Window window,
        spec_an::Detector detector)
        : Message{ID::SpecAnConfig},
          shared{shared},
          sampling_rate{sampling_rate},
          gen{gen},
          fft_log2n{fft_log2n},
          avg_blocks{avg_blocks},
          bin_first{bin_first},
          bins_per_slice{bins_per_slice},
          slices{slices},
          px_per_bin_num{px_per_bin_num},
          px_per_bin_den{px_per_bin_den},
          window{window},
          detector{detector} {
    }

    spec_an::Shared* shared;
    uint32_t sampling_rate;
    uint16_t gen;
    uint8_t fft_log2n;
    uint16_t avg_blocks;     /* FFTs power-averaged per slice */
    int16_t bin_first;       /* natural FFT bin of a slice's first bin, mod N */
    uint16_t bins_per_slice; /* K: slice s covers global bins [s*K, (s+1)*K) */
    uint16_t slices;
    /* Display pixels per FFT bin, as a ratio: fs * (points - 1) / (N * span). */
    uint64_t px_per_bin_num;
    uint64_t px_per_bin_den;
    spec_an::Window window;
    spec_an::Detector detector;
};

/* M4 -> M0: samples for request `seq` are in, the tuner is free. */
class SpecAnCapturedMessage : public Message {
   public:
    constexpr SpecAnCapturedMessage(uint16_t gen, uint32_t seq)
        : Message{ID::SpecAnCaptured},
          gen{gen},
          seq{seq} {
    }

    uint16_t gen;
    uint32_t seq;
};

/* M4 -> M0: display points [px_begin, px_end) of pix[sweep & 1] are final. */
class SpecAnSliceMessage : public Message {
   public:
    constexpr SpecAnSliceMessage(
        uint16_t gen,
        uint16_t slice,
        uint8_t sweep,
        bool last,
        uint16_t px_begin,
        uint16_t px_end)
        : Message{ID::SpecAnSlice},
          gen{gen},
          slice{slice},
          sweep{sweep},
          last{last},
          px_begin{px_begin},
          px_end{px_end} {
    }

    uint16_t gen;
    uint16_t slice;
    uint8_t sweep; /* req_sweep of the capture; pix[sweep & 1] holds the points */
    bool last;
    uint16_t px_begin;
    uint16_t px_end;
};

struct WMBusPacketMessage : public Message {
    constexpr WMBusPacketMessage()
        : Message{ID::WMBusPacketMessageID} {}

    uint16_t length = 0;
    uint8_t data[500] = {0};
};

/* GPS receiver (the GPS app's baseband images on the SD card, HackRF Pro).
 * M0 -> M4: start a snapshot capture and a search, start or stop tracking. */
struct GpsRxConfigMessage : public Message {
    constexpr GpsRxConfigMessage()
        : Message{ID::GpsRxConfig} {}
    enum Cmd : uint8_t { CMD_STOP = 0, /* stops a search and the tracking */
                         CMD_SEARCH = 1,
                         CMD_TRACK = 2, /* track the PRNs in prn_mask from the last search's results */
                         CMD_ADD = 3,   /* while tracking: aided search for one newly risen satellite (add_*) */
                         CMD_GAL = 4 }; /* while tracking: aided Galileo E1 search (add_*; codes in gal_codes of the status);
                                         * add_prn 0: Galileo off (search stopped, Galileo channels released) */
    uint8_t cmd{CMD_STOP};
    uint8_t k_ms{20};      /* snapshot length, 1..20 ms */
    uint8_t half_bins{12}; /* Doppler bins of 500 Hz either side of each centre */
    uint8_t refine{1};     /* refine detected PRNs: 1 Doppler + code (snapshot fix), 2 Doppler only (fast, tracking hand-over) */
    uint32_t prn_mask{0};  /* bit p-1: search PRN p */
    int16_t center[32]{};  /* Doppler window centre per PRN, Hz */
    uint8_t confirm{1};    /* report a PRN only if a second snapshot confirms it */
    uint8_t track{0};      /* CMD_SEARCH: hand the confirmed PRNs to the tracking loops when done ("Autonomous") */
    uint8_t max_ch{8};     /* tracking channels at most (M4 time) */
    /* CMD_TRACK (tracking image): age of the search results, ms on the app's
     * clock (the acquisition image took the result snapshot before the image
     * switch; the code phases are propagated over this time) */
    uint32_t age_ms{0};
    /* CMD_ADD: PRN, predicted code phase (chips) of the signal arriving at
     * sample add_at (the image's sample count, as in the measurement sets),
     * predicted Doppler incl. the receiver clock (Hz), windows of 15 half chips
     * either side to search */
    uint8_t add_prn{0};
    uint8_t add_windows{0};
    float add_code{0};
    float add_doppler{0};
    uint64_t add_at{0};
    /* CMD_GAL: Galileo channels the detection may go to (gal_acq::kMaxTrk at
     * most; the M4 time decides too: GPS first); 0 = search only (Heard) */
    uint8_t gal_max{0};
};

/* M4 -> M0: progress, noise statistics, and where the results and the lent
 * RAM are (in the M4 RAM block kept across the GPS images, gps_keep.hpp). */
struct GpsRxStatusMessage : public Message {
    constexpr GpsRxStatusMessage()
        : Message{ID::GpsRxStatus} {}
    enum State : uint8_t { IDLE = 0,
                           CAPTURING,
                           SEARCHING,
                           REFINING,
                           DONE,
                           TRACKING }; /* tracking loops running (results of the search that started them stay valid) */
    enum Level : uint8_t { LEVEL_OK = 0,
                           LEVEL_LOW, /* snapshot AC rms below the floor: no search done */
                           LEVEL_HIGH /* clipping: results still reported */ };
    /* The M4 RAM the GPS images lend the app: orbit tables and solver work
     * space (never touched by the M4). */
    static constexpr uint16_t kScratchSize = 25344; /* orbit tables, Autonomous (AutoState, PVT work), the log ring, the other systems' sky */
    uint8_t state{IDLE};
    uint8_t level{LEVEL_OK};
    uint8_t confirming{0}; /* second (confirmation) snapshot under way */
    uint8_t done_prn{0};   /* PRNs searched */
    uint8_t total_prn{0};  /* PRNs requested */
    uint8_t k_ms{0};
    uint16_t rms_x10{0};          /* noise rms of the snapshot, LSB x 10 */
    uint16_t clip_permille{0};    /* samples at full scale, per mille */
    uint32_t seq{0};              /* search number (results belong to it) */
    uint32_t unit_cycles{0};      /* average M4 cycles of one work unit (one 1 ms block of one Doppler bin) */
    uint32_t units{0};            /* work units done in this search */
    const void* results{nullptr}; /* gps_rx::Result[32], by PRN - 1 */
    uint8_t* scratch{nullptr};
    uint16_t scratch_size{0};
    const void* track{nullptr}; /* gps_trk::Shared (channels, subframes, measurements), written by the tracking image */
    int32_t dt_ms{0};           /* the confirmation interval: snapshot B - snapshot A, ms (diagnostics) */
    /* the image's sample clock (2.048 Msps, restarts with each image): the next sample
     * when this status was sent, and the first sample of the snapshot the
     * results refer to (their age; the app maps samples to its own clock) */
    uint64_t sample_now{0};
    uint64_t results_sample{0};
    uint8_t image{0}; /* the GPS image sending this (gps_image::Id: 1 L1 acquisition, 2 L1 tracking) */
    /* tracking image: the aided Galileo E1 search (gal_acq.hpp). The app puts a
     * satellite's codes in gal_codes (gal_acq::Codes, 1 KB) before CMD_GAL;
     * gal_result (gal_acq::Result) belongs to search number gal_seq. Both live in
     * the tracking image's RAM: valid while it runs. */
    void* gal_codes{nullptr};
    const void* gal_result{nullptr};
    uint16_t gal_seq{0};
    uint8_t gal_busy{0};
    /* tracking image: the Galileo E1 channels (gal_acq::TrkShared, its RAM) and
     * RAM it lends the app's I/NAV decoder (gal_acq::kNavWorkBytes) */
    const void* gal_track{nullptr};
    void* gal_nav{nullptr};
};

#endif /*__MESSAGE_H__*/
