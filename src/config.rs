// This file is part of the rust-radio-sx1231 project.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// Copyright 2020-2021 Erik Henriksson


use crate::register::*;

use radio::ReceiveInfo;

/// RX/TX radio channel configuration
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serde", serde(default))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Channel {
    /// (G)FSK frequency in Hz (defaults to 868 MHz)
    pub freq: u32,

    /// (G)FSK  channel baud-rate (defaults to 50kbps)
    pub br: u32,

    /// (G)FSK channel bandwidth (defaults to 125kHz)
    pub bw: Bandwidth,

    /// (G)FSK AFC channel bandwidth (defaults to 125kHz)
    pub bw_afc: Bandwidth,

    /// Frequency deviation in Hz (defaults to 50kHz)
    pub fdev: u32,
}

impl Default for Channel {
    fn default() -> Self {
        Self {
            freq: 868_000_000,
            br: 50_000,
            bw: Bandwidth::Bw125000,
            bw_afc: Bandwidth::Bw125000,
            fdev: 50_000,
        }
    }
}

/// Sx1231 radio configuration
#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Preamble length in symbols (defaults to 0x8)
    pub preamble_len: u16,

    /// Payload length configuration (defaults to Variable / Explicit header mode)
    pub payload_mode: PayloadMode,

    /// DC-Free encoding/decoding
    pub dc_free: DcFree,

    /// Payload RX CRC configuration (defaults to enabled)
    pub crc: Crc,

    /// Disable auto-clear FIFO and restart RX on CRC failure
    pub crc_auto_clear: CrcAutoClear,

    /// Address filtering in RX mode
    pub address_filter: AddressFilter,

    /// Set modulation
    pub modulation: ModulationType,

    /// Set data processing mode
    pub data_mode: DataMode,

    /// Sync word
    pub sync_word: &'static [u8],

    /// Node address for filtering
    pub node_address: u8,

    /// Broadcast address for filtering
    pub broadcast_address: u8,

    /// RX continuous mode
    pub rx_continuous: bool,

    /// Device channel configuration
    pub channel: Channel,

    /// Device power amplifier configuration
    pub power: i8,

    /// Device crystal frequency (defaults to 32MHz)
    pub xtal_freq: u32,

    /// Device frequency step.
    pub freq_step: f32,

    /// Number of loops for internally blocking radio operations
    pub timeout_ticks: u32,

    /// Specifies the RSSI threshold to use.
    pub rssi_threshold: u8,

    /// Enables automatic Rx restart (RSSI phase) after PayloadReady occurred
    /// and packet has been completely read from FIFO.
    pub auto_rx_restart: AutoRxRestart,

    /// After PayloadReady occurred, defines the delay between FIFO empty and
    /// the start of a new RSSI phase for next packet. Must match the
    /// transmitter’s PA ramp-down time. Defaults to 4.
    /// - Tdelay = 0 if inter_packet_rx_delay >= 12
    /// - Tdelay = (2^inter_packet_rx_delay) / bit_rate otherwise
    pub inter_packet_rx_delay: u8,

    /// Defines the condition to start packet transmission.
    pub tx_start_condition: TxStartCondition,

    /// Used to trigger FifoLevel interrupt.
    pub fifo_threshold: u8,

    /// Improved AFC routine for signals with modulation index lower than 2.
    pub afc_low_beta: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            preamble_len: 0x8,
            payload_mode: PayloadMode::Variable,
            dc_free: DcFree::Off,
            crc: Crc::Off,
            crc_auto_clear: CrcAutoClear::Off,
            address_filter: AddressFilter::Off,
            modulation: ModulationType::Fsk,
            data_mode: DataMode::Packet,
            sync_word: &[],
            node_address: 0,
            broadcast_address: 0,
            rx_continuous: false,
            channel: Channel::default(),
            power: 13,
            xtal_freq: 32_000_000u32,
            freq_step: 61.03515625,
            timeout_ticks: 2048,
            rssi_threshold: 200, // -100 dBm
            auto_rx_restart: AutoRxRestart::On,
            inter_packet_rx_delay: 4,
            tx_start_condition: TxStartCondition::FifoNotEmpty,
            fifo_threshold: 32,
            afc_low_beta: true,
        }
    }
}

/// Payload length mode configuration
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PayloadMode {
    /// Constant length payloads use implicit headers
    Constant(u16),
    /// Variable length payloads must be contain explicit headers
    Variable,
}

/// Received packet information
#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PacketInfo {
  /// Received Signal Strength Indication
  pub rssi: i16,
  /// Signal to Noise Ratio
  pub snr: Option<i16>,
}

impl Default for PacketInfo {
  fn default() -> Self {
      Self { rssi: 0, snr: None }
  }
}

impl ReceiveInfo for PacketInfo {
    fn rssi(&self) -> i16 {
        self.rssi
    }
}