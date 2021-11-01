//! Sx1231 Integration testing
//
// This file is part of the rust-radio-sx1231 project.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// Copyright 2019 Ryan Kurte
// Copyright 2020-2021 Erik Henriksson

use std::thread;
use std::time::{Duration, Instant};

use core::convert::Infallible;

use nb;
use void::Void;

extern crate radio_sx1231;
use radio_sx1231::prelude::*;
use radio_sx1231::register::{Bandwidth, ModulationType};

extern crate radio;
use radio::{Receive, State, Transmit};

extern crate linux_embedded_hal;
use linux_embedded_hal::sysfs_gpio::Direction;
use linux_embedded_hal::{Delay, Pin as Pindev};

extern crate bitbang_hal;
use bitbang_hal::spi::{SPI, MODE_0};

extern crate embedded_hal;
use embedded_hal::timer::{CountDown, Periodic};

extern crate shared_bus;
use shared_bus::BusManagerSimple;

extern crate log;
use log::{debug};
extern crate simplelog;
use simplelog::{TermLogger, TerminalMode, LevelFilter};


pub fn load_spi() -> SPI<Pindev, Pindev, Pindev, SysTimer> {
    let miso = load_pin(9, Direction::In);
    let mosi = load_pin(10, Direction::Out);
    let clk = load_pin(11, Direction::Out);
    let mut timer = SysTimer::new();
    timer.start(Duration::from_micros(10)); // 50kHz SPI clock
    let mut spi = SPI::new(MODE_0, miso, mosi, clk, timer);
    spi
}

#[test]
fn integration() {
    let mut log_config = simplelog::ConfigBuilder::new();
    log_config.add_filter_ignore("embedded_spi".to_string());
    log_config.set_location_level(LevelFilter::Off);

    TermLogger::init(LevelFilter::Trace, log_config.build(), TerminalMode::Mixed).unwrap();

    // Fetch configuration file name
    let config_file = match std::env::var("SX1231_TEST_CONFIG") {
        Ok(v) => v,
        Err(_e) => "configs/pi-ci-sx1231.toml".to_owned(),
    };

    println!("Using configuration file: {}", config_file);

    let mut config = Config::default();
    config.channel.freq = 868_750_000;
    config.channel.br = 100_000;
    config.channel.fdev = 100_000;
    config.channel.bw = Bandwidth::Bw166700;
    config.channel.bw_afc = Bandwidth::Bw200000;
    config.preamble_len = 8;
    config.payload_mode = PayloadMode::Variable;
    config.modulation = ModulationType::Gfsk05;
    config.power = 0;

    let mut spi = load_spi();
    let mut spi_proxy = BusManagerSimple::new(spi);

    let cs0 = load_pin(8, Direction::Out);
    let busy0 = load_pin(22, Direction::In);
    let rst0 = load_pin(24, Direction::Out);
    let mut radio1 = Sx1231::new(spi_proxy.acquire_spi(), cs0, busy0, rst0, Delay {}, &config).expect("error creating radio1");

    let cs1 = load_pin(7, Direction::Out);
    let busy1 = load_pin(23, Direction::In);
    let rst1 = load_pin(25, Direction::Out);
    let mut radio2 = Sx1231::new(spi_proxy.acquire_spi(), cs1, busy1, rst1, Delay {}, &config).expect("error creating radio2");

    // Load configurations from file
    // let configs = load_config::<HashMap<String, DeviceConfig>>(&config_file);

    // let config1 = configs.get("radio-0").expect("Missing radio-0 object");
    // let miso = load_pin(9, Direction::In);

    // let config2 = configs.get("radio-1").expect("Missing radio-1 object");

    // let (w1, w2) = (config1.load_with_busy_ready(), config2.load_with_busy_ready());

    // let mut radio1 = Sx1231::new(w1, &Config::default()).expect("error creating radio1");
    // let mut radio2 = Sx1231::new(w2, &Config::default()).expect("error creating radio2");

    println!("Testing send/receive");

    // let data = &[0xaa, 0xbb, 0xcc];
    let data = &[1, 152, 1, 173, 222, 2, 0, 173, 222, 1, 0, 222, 173];
    // Configure receive
    radio1.start_receive().unwrap();

    thread::sleep(Duration::from_millis(1)); // Wait for RX to start.

    radio1.check_receive(false).unwrap();

    // Start transmit
    radio2.start_transmit(data).unwrap();

    // Poll on tx and rx complete
    let mut sent = false;
    let mut received = false;
    let mut buff = [0u8; 128];
    let mut n = 0;
    let mut info = PacketInfo::default();

    for _i in 0..10 {
        // Check RX state
        if radio1.check_receive(false).unwrap() && !received {
            n = radio1.get_received(&mut info, &mut buff).unwrap();
            received = true;
            println!("RX complete ({:?} {:?})", info, &buff[..n]);
        }

        // Check TX state
        if radio2.check_transmit().unwrap() && !sent {
            println!("TX complete");
            sent = true;
        }

        if (sent && received) {
            break;
        }

        thread::sleep(Duration::from_millis(100));
    }
    radio1.set_state(ModemMode::Standby);
    radio2.set_state(ModemMode::Standby);

    assert!(sent, "Send not completed");
    assert!(received, "Receive not completed");
    assert_eq!(data, &buff[..n]);
}

fn load_pin(index: u64, direction: Direction) -> Pindev {
    debug!(
        "Connecting to pin: {} with direction: {:?}",
        index, direction
    );

    let p = Pindev::new(index);
    p.export().expect("error exporting cs pin");
    p.set_direction(direction)
        .expect("error setting cs pin direction");

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