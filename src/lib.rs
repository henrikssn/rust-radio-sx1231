//! Sx1231 Rust Radio Driver
//!
//! This library implements a driver for the Semtec Sx1231 series of sub-GHz ISM band Radio ICs.
//!
//! For use with [embedded-hal](https://github.com/rust-embedded/embedded-hal) implementations.
//!
// This file is part of the rust-radio-sx1231 project.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// Copyright 2018 Ryan Kurte
// Copyright 2020-2021 Erik Henriksson

#![no_std]

use core::cmp::{max, min};

#[cfg(feature = "log")]
use log::{debug, trace, warn};

#[cfg(feature = "defmt")]
use defmt::{debug, trace, warn};

#[cfg(feature = "serde")]
extern crate serde;

use core::fmt::Debug;
use core::marker::PhantomData;

extern crate embedded_hal as hal;
use hal::blocking::spi::{Transfer, Write};
use hal::digital::v2::OutputPin;

extern crate radio;
use radio::{
    Channel as _, Interrupts as _, Power as _, Register as _, Registers as _, Rssi as _, State as _,
};

pub mod prelude;
use prelude::*;

pub mod register;
use register::*;

pub mod config;

/// Sx1231 device object
///
/// Operating functions are implemented as traits from the [radio] package
///
pub struct Sx1231<Spi, CsPin, SpiError, PinError> {
    /// SPI device
    spi: Spi,

    /// Chip select pin (required)
    cs: CsPin,

    rx_buf: [u8; 256],
    rx_buf_len: usize,

    _se: PhantomData<SpiError>,
    _pe: PhantomData<PinError>,

    config: Config,
}

/// Sx1231 error type
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<SpiError, PinError> {
    /// SPI error
    Spi(SpiError),
    /// Pin control error
    Pin(PinError),
    /// Invalid configuration
    InvalidConfiguration,
    /// Transaction aborted
    Aborted,
    /// Invalid response from device
    InvalidResponse,
    /// Timeout while awaiting operation completion
    Timeout,
    /// incoming packet CRC error
    Crc,
    /// Received packet exceeds buffer size
    BufferSize(usize),
    /// Invalid or unrecognised device
    InvalidDevice(u8),
    /// Packet size disagrees with FIFO interrupt (first is packet len, second is buffer size)
    InvalidPacketSize(usize, usize),
    // #[error("Unexpected register value (address: 0x{0:02x})")]
    UnexpectedValue(u8),
}

impl<Spi, CsPin, SpiError, PinError> Sx1231<Spi, CsPin, SpiError, PinError>
where
    Spi: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    /// Create a new radio instance with the provided SPI implementation and pins
    pub fn new(spi: Spi, cs: CsPin) -> Self {
        Sx1231 {
            spi,
            cs,
            rx_buf: [0u8; 256],
            rx_buf_len: 0,
            _se: PhantomData,
            _pe: PhantomData,
            config: Config::default(),
        }
    }

    /// Return hardware resources for reuse
    pub fn free(self) -> (Spi, CsPin) {
        (self.spi, self.cs)
    }
}

impl<Spi, CsPin, SpiError, PinError> Sx1231<Spi, CsPin, SpiError, PinError>
where
    Spi: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    /// (re)apply device configuration
    pub fn configure(&mut self, config: &Config) -> Result<(), Error<SpiError, PinError>> {
        // Read version to make sure SPI bus works correctly
        let version = self.silicon_version()?;
        if version & 0xF0 != 0x20 {
            return Err(Error::InvalidDevice(version));
        }

        // // Set channel configuration
        self.set_channel(&config.channel)?;

        self.write_regs(
            DataModulation::ADDRESS,
            &DataModulation::new()
                .with_mod_type(config.modulation)
                .with_mode(config.data_mode)
                .into_bytes(),
        )?;

        // // Set preamble length
        let preamble_len = [
            (config.preamble_len >> 8) as u8,
            (config.preamble_len & 0xFF) as u8,
        ];
        self.write_regs(Preamble::ADDRESS, &preamble_len)?;

        self.set_sync_word(config.sync_word)?;

        // Set payload length
        let packet_format = match config.payload_mode {
            PayloadMode::Constant(v) => {
                self.write_reg(PayloadLength::ADDRESS, v as u8)?;
                PacketFormat::Fixed
            }
            PayloadMode::Variable => {
                self.write_reg(PayloadLength::ADDRESS, 0xFF)?;
                PacketFormat::Variable
            }
        };

        // Set packetconfig1 register
        self.write_regs(
            PacketConfig1::ADDRESS,
            &PacketConfig1::new()
                .with_packet_format(packet_format)
                .with_dc_free(config.dc_free)
                .with_crc(config.crc)
                .with_crc_auto_clear(config.crc_auto_clear)
                .with_address_filter(config.address_filter)
                .into_bytes(),
        )?;

        // Set packetconfig2 register
        self.write_regs(
            PacketConfig2::ADDRESS,
            &PacketConfig2::new()
                .with_auto_rx_restart(config.auto_rx_restart)
                .with_inter_packet_rx_delay(config.inter_packet_rx_delay)
                .into_bytes(),
        )?;

        // Configure TXStart
        self.write_regs(
            FifoThreshold::ADDRESS,
            &FifoThreshold::new()
                .with_start_condition(config.tx_start_condition)
                .with_threshold(config.fifo_threshold)
                .into_bytes(),
        )?;

        self.write_regs(
            RssiThreshold::ADDRESS,
            &RssiThreshold::new()
                .with_value(config.rssi_threshold)
                .into_bytes(),
        )?;

        self.write_regs(
            AfcControl::ADDRESS,
            &AfcControl::new()
                .with_low_beta_on(config.afc_low_beta)
                .into_bytes(),
        )?;

        if config.afc_low_beta {
            self.write_regs(
                0x6f, // RegTestDagc
                &[0x20],
            )?; // Improved margin, use if AfcLowBetaOn=1
            self.write_regs(
                0x71, // RegTestAfc
                &[20],
            )?; // LowBetaAfcOffset = 20 * 488 = 9.8kHz (should be ~10% of RxBw and > DCC cutoff)
        }

        self.write_regs(
            AfcFei::ADDRESS,
            &AfcFei::new().with_afc_auto_on(true).into_bytes(),
        )?;

        self.write_regs(
            0x2bu8, //timeout rssi threshold
            &[64u8],
        )?; // 64 * 16 * 10 us = 12ms

        self.write_regs(
            Lna::ADDRESS,
            &Lna::new()
                .with_zin(LnaZin::Zin50Ohm)
                .with_gain_select(LnaGainSelect::Agc)
                .into_bytes(),
        )?;

        self.write_regs(
            0x58, // RegTestLna
            &[0x2d],
        )?; // SensitivityBoost

        // Configure power amplifier
        self.set_power(config.power)?;

        // Update config
        self.config = config.clone();

        Ok(())
    }

    fn set_sync_word(&mut self, sync_word: &[u8]) -> Result<(), Error<SpiError, PinError>> {
        if !sync_word.is_empty() {
            self.write_regs(
                SyncConfig::ADDRESS,
                &SyncConfig::new()
                    .with_sync_on(true)
                    .with_size_minus_one(sync_word.len() as u8 - 1)
                    .into_bytes(),
            )?;
            self.write_regs(SyncValue::ADDRESS, sync_word)?;
        } else {
            self.write_regs(
                SyncConfig::ADDRESS,
                &SyncConfig::new().with_sync_on(false).into_bytes(),
            )?;
        }
        Ok(())
    }

    /// Fetch device silicon version
    pub fn silicon_version(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register::<Version>().map(|r| r.into())
    }

    // Calculate a channel number from a given frequency
    pub(crate) fn freq_to_channel_index(&self, freq: u32) -> u32 {
        let step = (self.config.xtal_freq as f32) / (2u32.pow(19) as f32);
        let ch = (freq as f32) / step;
        ch as u32
    }

    // Set the channel by frequency
    pub fn set_frequency(&mut self, freq: u32) -> Result<(), Error<SpiError, PinError>> {
        let channel = self.freq_to_channel_index(freq);

        let outgoing = [(channel >> 16) as u8, (channel >> 8) as u8, channel as u8];

        debug!("Set channel to index: {:?} (freq: {:?})", channel, freq);

        self.write_regs(CarrierFreq::ADDRESS, &outgoing)?;

        Ok(())
    }

    /// Read a u8 value from the specified register
    pub fn read_reg<R>(&mut self, reg: R) -> Result<u8, Error<SpiError, PinError>>
    where
        R: Copy + Clone + Debug + Into<u8>,
    {
        let mut incoming = [0u8; 1];
        self.read_regs(reg.into(), &mut incoming)?;
        Ok(incoming[0])
    }

    /// Write a u8 value to the specified register
    pub fn write_reg<R>(&mut self, reg: R, value: u8) -> Result<(), Error<SpiError, PinError>>
    where
        R: Copy + Clone + Debug + Into<u8>,
    {
        self.write_regs(reg.into(), &[value])?;

        Ok(())
    }

    /// Update the specified register with the provided value & mask
    pub fn update_reg<R>(
        &mut self,
        reg: R,
        mask: u8,
        value: u8,
    ) -> Result<(), Error<SpiError, PinError>>
    where
        R: Copy + Clone + Debug + Into<u8>,
    {
        let existing = self.read_reg(reg)?;
        let updated = (existing & !mask) | (value & mask);
        self.write_reg(reg, updated)?;

        Ok(())
    }

    /// Read from the specified register
    fn read_regs<R>(&mut self, reg: R, data: &mut [u8]) -> Result<(), Error<SpiError, PinError>>
    where
        R: Copy + Clone + Debug + Into<u8>,
    {
        self.cs.set_low().map_err(|e| Error::Pin(e))?;

        // Send command then read data
        self.spi
            .write(&[reg.into() & 0x7F])
            .map_err(|e| Error::Spi(e))?;
        self.spi.transfer(data).map_err(|e| Error::Spi(e))?;

        self.cs.set_high().map_err(|e| Error::Pin(e))?;

        Ok(())
    }

    /// Write to the specified register
    fn write_regs<R>(&mut self, reg: R, data: &[u8]) -> Result<(), Error<SpiError, PinError>>
    where
        R: Copy + Clone + Debug + Into<u8>,
    {
        self.cs.set_low().map_err(|e| Error::Pin(e))?;
        #[cfg(feature = "log")]
        trace!("Write reg:  {:?} (0x{:02x}): {:x?}", reg, reg.into(), data);
        #[cfg(feature = "defmt")]
        trace!("Write reg:  {} (0x{}): {}", reg, reg.into(), data);

        // Send command then write data
        self.spi
            .write(&[reg.into() | 0x80])
            .map_err(|e| Error::Spi(e))?;
        self.spi.write(data).map_err(|e| Error::Spi(e))?;

        self.cs.set_high().map_err(|e| Error::Pin(e))?;

        Ok(())
    }

    /// Write to the specified buffer
    fn write_fifo(&mut self, data: &[u8]) -> Result<(), Error<SpiError, PinError>> {
        self.cs.set_low().map_err(|e| Error::Pin(e))?;

        // Setup FIFO buffer write
        self.spi
            .write(&[Fifo::ADDRESS | 0x80])
            .map_err(|e| Error::Spi(e))?;

        // Data
        self.spi.write(data).map_err(|e| Error::Spi(e))?;

        self.cs.set_high().map_err(|e| Error::Pin(e))?;

        Ok(())
    }

    /// Read from the specified buffer
    fn read_fifo(&mut self, len: usize) -> Result<(), Error<SpiError, PinError>> {
        let rx_buf_len = self.rx_buf_len;
        if rx_buf_len + len > self.rx_buf.len() {
            return Err(Error::BufferSize(rx_buf_len + len));
        } else if len > self.rx_buf.len() {
            // Addition above might overflow
            return Err(Error::BufferSize(len));
        }

        self.cs.set_low().map_err(|e| Error::Pin(e))?;

        // Setup FIFO buffer read
        self.spi
            .write(&[Fifo::ADDRESS])
            .map_err(|e| Error::Spi(e))?;

        // Read the rest of the FIFO data
        self.spi
            .transfer(&mut self.rx_buf[rx_buf_len..(rx_buf_len + len)])
            .map_err(|e| Error::Spi(e))?;
        self.rx_buf_len += len;

        self.cs.set_high().map_err(|e| Error::Pin(e))?;

        Ok(())
    }
}

impl<Spi, CsPin, SpiError, PinError> Sx1231<Spi, CsPin, SpiError, PinError>
where
    Spi: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    pub fn set_state_checked(&mut self, state: ModemMode) -> Result<(), Error<SpiError, PinError>> {
        // Send set state command
        trace!("Set state to: {:?} (0x{:02x})", state, state as u8);
        self.set_state(state)?;

        let mut ticks = 0;
        loop {
            // Fetch current state
            let s = self.get_state()?;
            trace!("Received: {:?}", s);

            // Check for expected state
            if state == s {
                break;
            }
            // Timeout eventually
            if ticks >= self.config.timeout_ticks {
                warn!("Set state timeout: {:?}, {:?}", state, s);
                return Err(Error::Timeout);
            }
            ticks += 1;
        }
        Ok(())
    }
}

impl<Spi, CsPin, SpiError, PinError> radio::State for Sx1231<Spi, CsPin, SpiError, PinError>
where
    Spi: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    type State = ModemMode;
    type Error = Error<SpiError, PinError>;

    /// Fetch device modem mode
    fn get_state(&mut self) -> Result<Self::State, Self::Error> {
        let op_mode = OpMode::from_bytes([self.read_reg(OpMode::ADDRESS)?]);
        Ok(op_mode.modem_mode())
    }

    /// Set device modem mode
    fn set_state(&mut self, state: Self::State) -> Result<(), Self::Error> {
        let mut op_mode = self.read_register::<OpMode>()?;
        op_mode.set_modem_mode(state);
        self.write_regs(OpMode::ADDRESS, &op_mode.into_bytes())?;
        Ok(())
    }
}

impl<Spi, CsPin, SpiError, PinError> radio::Power for Sx1231<Spi, CsPin, SpiError, PinError>
where
    Spi: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    type Error = Error<SpiError, PinError>;

    /// Set transmit power (using the existing `PaConfig`)
    fn set_power(&mut self, power: i8) -> Result<(), Error<SpiError, PinError>> {
        // Limit to viable input range
        let power = min(max(power, 0), 20) as u8;

        let mut config = PaLevel::new();

        // See https://andrehessling.de/2015/02/07/figuring-out-the-power-level-settings-of-hoperfs-rfm69-hwhcw-modules/
        if power < 14 {
            config.set_pa1_on(true);
            config.set_level(power + 18);
        } else if power < 18 {
            config.set_pa1_on(true);
            config.set_pa2_on(true);
            config.set_level(power + 14);
        }

        debug!("Updated PA config for {} dBm: {:?}", power, config);

        self.write_regs(PaLevel::ADDRESS, &config.into_bytes())?;

        Ok(())
    }
}

impl<Spi, CsPin, SpiError, PinError> radio::Interrupts for Sx1231<Spi, CsPin, SpiError, PinError>
where
    Spi: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    type Irq = IrqFlags;
    type Error = Error<SpiError, PinError>;

    /// Fetch pending interrupts from the device
    /// If the clear option is set, this will also clear any pending flags
    fn get_interrupts(&mut self, clear: bool) -> Result<Self::Irq, Self::Error> {
        let mut regs = [0u8; 2];
        self.read_regs(IrqFlags::ADDRESS, &mut regs)?;
        let irq = IrqFlags::from_bytes(regs);

        if clear {
            self.write_regs(
                IrqFlags::ADDRESS,
                &irq.with_rssi(false).with_fifo_overrun(false).into_bytes(),
            )?;
        }

        Ok(irq)
    }
}

impl<Spi, CsPin, SpiError, PinError> radio::Channel for Sx1231<Spi, CsPin, SpiError, PinError>
where
    Spi: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    type Channel = Channel;
    type Error = Error<SpiError, PinError>;

    /// Set the channel for future receive or transmit operations
    fn set_channel(&mut self, channel: &Channel) -> Result<(), Error<SpiError, PinError>> {
        // Set frequency
        self.set_frequency(channel.freq)?;

        // Need our own rounding alogrithm because corelib doesn't have
        // f32::round
        let round = |x: f32| -> u32 {
            let integer = x as u32;
            if (x - (integer as f32)) < 0.5 {
                integer
            } else {
                integer + 1
            }
        };

        // Calculate channel configuration
        let fdev = round((channel.fdev as f32) / self.config.freq_step) as u16;
        let datarate = round(self.config.xtal_freq as f32 / channel.br as f32) as u16;
        trace!("fdev: {} bitrate: {}", fdev, datarate);

        // Set frequency deviation
        self.write_regs(FreqDev::ADDRESS, &[(fdev >> 8) as u8, (fdev & 0xFF) as u8])?;
        // self.write_regs(FreqDev::ADDRESS, FreqDev::new().with_freq_dev(fdev).into_bytes())?;

        // Set bitrate
        // self.write_regs(Bitrate::ADDRESS, Bitrate::new().with_bitrate(datarate).into_bytes())?;
        self.write_regs(
            Bitrate::ADDRESS,
            &[(datarate >> 8) as u8, (datarate & 0xFF) as u8],
        )?;

        // Set bandwidths
        self.write_reg(RxBw::ADDRESS, 0b0100_0000 | channel.bw as u8)?;
        self.write_reg(AfcBw::ADDRESS, 0b0100_0000 | channel.bw_afc as u8)?;

        Ok(())
    }
}

impl<Spi, CsPin, SpiError, PinError> radio::Transmit for Sx1231<Spi, CsPin, SpiError, PinError>
where
    Spi: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    type Error = Error<SpiError, PinError>;

    /// Start sending a packet
    fn start_transmit(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        #[cfg(feature = "log")]
        debug!("Starting send (data: {:x?}, len: {})", data, data.len());
        #[cfg(feature = "defmt")]
        debug!("Starting send (data: {}, len: {})", data, data.len());

        // TODO: support large packet sending
        assert!(data.len() < 255);

        self.set_state_checked(ModemMode::Standby)?;

        let irq = self.get_interrupts(true)?;
        trace!("clearing interrupts {:?}", irq);

        // Length
        match self.config.payload_mode {
            PayloadMode::Constant(_) => {}
            PayloadMode::Variable => {
                self.write_fifo(&[data.len() as u8])?;
            }
        }

        self.write_fifo(&data[..min(64, data.len())])?;

        // This only works if RX leaves radio in Standby/Sleep state, otherwise
        // there might be garbage in the FIFO (see RFM69 datasheet page 46).
        self.set_state(ModemMode::Transmitter)?;

        if data.len() > 64 {
            for chunk in data[64..].chunks(32) {
                // Wait for FIFO to contains less than 32 elements.
                while self.get_interrupts(false)?.fifo_level() {}
                // Write data to FIFO
                self.write_fifo(chunk)?;
            }
        }

        Ok(())
    }

    /// Check for transmission completion
    /// This method should be polled (or checked following and interrupt) to indicate sending
    /// has completed
    fn check_transmit(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        // Fetch interrupts
        let irq = self.get_interrupts(true)?;

        trace!("Check transmit IRQ: {:?}", irq);

        // Check for completion
        if irq.packet_sent() {
            self.set_state_checked(ModemMode::Sleep)?;
            return Ok(true);
        }

        Ok(false)
    }
}

impl<Spi, CsPin, SpiError, PinError> radio::Receive for Sx1231<Spi, CsPin, SpiError, PinError>
where
    Spi: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    type Info = PacketInfo;
    type Error = Error<SpiError, PinError>;

    /// Start receive mode
    fn start_receive(&mut self) -> Result<(), Self::Error> {
        trace!("Starting receive");

        // Revert to standby state
        self.set_state_checked(ModemMode::Standby)?;

        // Clear interrupts
        let irq = self.get_interrupts(true)?;
        trace!("clearing interrupts {:?}", irq);

        // Enter Rx mode (unchecked as we enter FsRx by default)
        self.set_state(ModemMode::Receiver)?;

        trace!("started receive");

        self.rx_buf_len = 0;

        Ok(())
    }

    /// Check receive state
    ///
    /// This returns true if a boolean indicating whether a packet has been received.
    /// The restart option specifies whether transient timeout or CRC errors should be
    /// internally handled (returning Ok(false)) or passed back to the caller as errors.
    fn check_receive(&mut self, restart: bool) -> Result<bool, Self::Error> {
        // Fetch interrupts
        let irq = self.get_interrupts(false)?;
        let s = self.get_state()?;
        let mut res = Ok(false);

        trace!("Check Receive (State: {:?}, IRQ: {:?})", s, irq);

        if irq.rssi() {
            let rssi = self.poll_rssi()?;
            trace!("Check Receive RSSI: {:?}", rssi);
        }

        if irq.payload_ready() {
            // Read out last part of packet.
            self.set_state_checked(ModemMode::Standby)?;
            if self.rx_buf_len == 0 {
                self.read_fifo(1)?;
            }
            let packet_len = self.rx_buf[0] as usize;
            if packet_len < self.rx_buf_len + 1 {
                // Something is terribly wrong, abort.
                return Err(Error::InvalidPacketSize(packet_len, self.rx_buf_len + 1));
            }
            // Read one byte extra as length byte does not count.
            self.read_fifo(packet_len - self.rx_buf_len + 1)?;
            debug!("RX complete! ({} bytes)", packet_len);
            res = Ok(true)
        } else if irq.fifo_level() {
            self.read_fifo(32)?;
            trace!(
                "Received chunk: {:?}",
                &self.rx_buf[(self.rx_buf_len - 32)..self.rx_buf_len]
            );
        } else if irq.timeout() {
            trace!("RX timeout");
            self.rx_buf_len = 0;
            res = Err(Error::Timeout);
        }

        // Handle restart logic
        match (restart, res) {
            (true, Err(_)) => {
                trace!("RX restarting");
                self.start_receive()?;
                Ok(false)
            }
            (_, r) => r,
        }
    }

    /// Fetch a received message
    ///
    /// This copies data into the provided slice, updates the provided information object,
    ///  and returns the number of bytes received on success
    fn get_received(&mut self, data: &mut [u8]) -> Result<(usize, Self::Info), Self::Error> {
        let mut info = PacketInfo::default();
        // Read the RSSI
        info.rssi = self.poll_rssi()?;
        // First byte is length and should not be returned.
        let packet_data = &self.rx_buf[1..self.rx_buf_len];
        data[..packet_data.len()].copy_from_slice(packet_data);
        self.rx_buf_len = 0;

        #[cfg(feature = "log")]
        debug!("Received data: {:02x?} info: {:?}", packet_data, &info);
        #[cfg(feature = "defmt")]
        debug!("Received data: {} info: {:?}", packet_data, &info);

        Ok((packet_data.len(), info))
    }
}

impl<Spi, CsPin, SpiError, PinError> radio::Rssi for Sx1231<Spi, CsPin, SpiError, PinError>
where
    Spi: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    type Error = Error<SpiError, PinError>;

    /// Poll for the current channel RSSI
    /// This should only be called in receive mode
    fn poll_rssi(&mut self) -> Result<i16, Error<SpiError, PinError>> {
        let rssi = -(self.read_reg(RssiValue::ADDRESS)? as i16) / 2;

        Ok(rssi)
    }
}

pub trait RegisterWord {
    type Bytes: Default + core::convert::AsMut<[u8]>;
    fn to_bytes(self) -> Self::Bytes;
    fn from_bytes(bytes: Self::Bytes) -> Self;
}

impl RegisterWord for u8 {
    type Bytes = [u8; 1];

    fn to_bytes(self) -> [u8; 1] {
        self.to_be_bytes()
    }

    fn from_bytes(bytes: [u8; 1]) -> Self {
        Self::from_be_bytes(bytes)
    }
}

impl RegisterWord for u16 {
    type Bytes = [u8; 2];

    fn to_bytes(self) -> [u8; 2] {
        self.to_be_bytes()
    }

    fn from_bytes(bytes: [u8; 2]) -> Self {
        Self::from_be_bytes(bytes)
    }
}

impl<Spi, CsPin, SpiError, PinError, Word, Bytes> radio::Registers<Word>
    for Sx1231<Spi, CsPin, SpiError, PinError>
where
    Spi: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
    Word: RegisterWord<Bytes = Bytes>,
    Bytes: Default + core::convert::AsMut<[u8]>,
{
    type Error = Error<SpiError, PinError>;

    fn read_register<R: radio::Register<Word = Word>>(
        &mut self,
    ) -> Result<R, Error<SpiError, PinError>> {
        let mut bytes = Bytes::default();

        self.cs.set_low().map_err(|e| Error::Pin(e))?;
        self.spi.write(&[R::ADDRESS]).map_err(|e| Error::Spi(e))?;
        self.spi
            .transfer(bytes.as_mut())
            .map_err(|e| Error::Spi(e))?;
        self.cs.set_high().map_err(|e| Error::Pin(e))?;

        let d = R::Word::from_bytes(bytes);
        R::try_from(d).map_err(|_| Error::UnexpectedValue(R::ADDRESS))
    }

    fn write_register<R: radio::Register<Word = Word>>(
        &mut self,
        r: R,
    ) -> Result<(), Error<SpiError, PinError>> {
        self.cs.set_low().map_err(|e| Error::Pin(e))?;
        self.spi.write(&[R::ADDRESS]).map_err(|e| Error::Spi(e))?;
        self.spi
            .write(r.into().to_bytes().as_mut())
            .map_err(|e| Error::Spi(e))?;
        self.cs.set_high().map_err(|e| Error::Pin(e))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
