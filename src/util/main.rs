//! Sx1231 command line utility
//!
//! Provides mechanisms for command line interaction with Sx1231 devices using linux spidev and sysfs_gpio
//
// This file is part of the rust-radio-sx1231 project.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// Copyright 2019 Ryan Kurte
// Copyright 2020-2021 Erik Henriksson

#[macro_use]
extern crate log;
extern crate simplelog;
use simplelog::{TermLogger, TerminalMode, LevelFilter};

use std::time::{Duration, Instant};

use core::convert::Infallible;

use nb;
use void::Void;

extern crate structopt;
use structopt::StructOpt;

extern crate humantime;

extern crate embedded_hal;
use embedded_hal::timer::{CountDown, Periodic};

extern crate linux_embedded_hal;
use linux_embedded_hal::sysfs_gpio::Direction;
use linux_embedded_hal::{Delay, Pin, Spidev};
use linux_embedded_hal::spidev;

extern crate radio;

extern crate radio_sx1231;
use radio_sx1231::prelude::*;
use radio_sx1231::register::{Bandwidth, ModulationType};

extern crate bitbang_hal;
use bitbang_hal::spi::{SPI, MODE_0};

mod options;
use options::*;

mod operations;
use operations::*;

fn main() {
    // Load options
    let opts = Options::from_args();

    // Setup logging
    let mut log_config = simplelog::ConfigBuilder::new();
    log_config.add_filter_ignore("embedded_spi".to_string());
    log_config.set_location_level(LevelFilter::Off);

    TermLogger::init(opts.level, log_config.build(), TerminalMode::Mixed).unwrap();

    debug!("Connecting to SPI device");
    let mut spi = load_spi();

    debug!("Configuring I/O pins");

    let cs = Pin::new(opts.cs);
    cs.export().expect("error exporting cs pin");
    cs.set_direction(Direction::Out)
        .expect("error setting cs pin direction");

    let rst = Pin::new(opts.rst);
    rst.export().expect("error exporting rst pin");
    rst.set_direction(Direction::Out)
        .expect("error setting rst pin direction");

    let busy = Pin::new(opts.busy);
    busy.export().expect("error exporting busy pin");
    busy.set_direction(Direction::Out)
        .expect("error setting busy pin direction");

    let ready = Pin::new(5);
    ready.export().expect("error exporting ready pin");
    ready.set_direction(Direction::Out)
        .expect("error setting busy pin direction");

    // Generate configuration
    debug!("Generating configuration");
    let mut config = Config::default();
    match &opts.command {
        Command::Gfsk(gfsk_config) => {
            let mut channel = Channel::default();

            if let Some(f) = gfsk_config.freq_mhz {
                channel.freq = f * 1_000_000;
            }

            config.channel = channel;
        },
        _ => (),
    }

    config.channel.freq = 868_750_000;
    config.channel.br = 100_000;
    config.channel.fdev = 100_000;
    config.channel.bw = Bandwidth::Bw166700;
    config.channel.bw_afc = Bandwidth::Bw200000;
    config.preamble_len = 8;
    config.payload_mode = PayloadMode::Variable;
    config.modulation = ModulationType::Gfsk05;
    config.power = 0;

    debug!("Creating radio instance");
    let mut radio =
        Sx1231::new(spi, cs, busy, rst, Delay {}, &config).expect("error creating device");

    debug!("Executing command");
    match opts.command {
        Command::SiliconVersion => {
            let version = radio
                .silicon_version()
                .expect("error fetching chip version");
            info!("Silicon version: 0x{:X}", version);
            return;
        }
        Command::Gfsk(gfsk_config) => {
            do_command(radio, gfsk_config.operation).expect("error executing command");
        }
    }
}

pub fn load_spi() -> SPI<Pin, Pin, Pin, SysTimer> {
    let miso = load_pin(9, Direction::In);
    let mosi = load_pin(10, Direction::Out);
    let clk = load_pin(11, Direction::Out);
    let mut timer = SysTimer::new();
    timer.start(Duration::from_micros(10)); // 50kHz SPI clock
    let mut spi = SPI::new(MODE_0, miso, mosi, clk, timer);
    spi
}

fn load_pin(index: u64, direction: Direction) -> Pin {
    debug!(
        "Connecting to pin: {} with direction: {:?}",
        index, direction
    );

    let p = Pin::new(index);
    p.export().expect("error exporting pin");
    p.set_direction(direction)
        .expect("error setting pin direction");

    p
}

pub struct SysTimer {
    start: Instant,
    duration: Duration,
}

impl SysTimer {
    /// Create a new timer instance.
    ///
    /// The `duration` will be initialized to 0, so make sure to call `start`
    /// with your desired timer duration before calling `wait`.
    pub fn new() -> SysTimer {
        SysTimer {
            start: Instant::now(),
            duration: Duration::from_millis(0),
        }
    }
}

impl CountDown for SysTimer {
    type Time = Duration;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        self.start = Instant::now();
        self.duration = count.into();
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if (Instant::now() - self.start) >= self.duration {
            // Restart the timer to fulfill the contract by `Periodic`
            self.start = Instant::now();
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl Periodic for SysTimer {}