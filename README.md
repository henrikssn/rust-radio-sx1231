# rust-radio-sx1231

A primarily rust driver (and command line utility) for the [Semtech SX1231] sub ghz ISM band radio IC.

[Semtech SX1231]: https://www.semtech.com/products/wireless-rf/frequency-shift-keying-fsk/sx1231

## Status

(G)FSK functionality working. OOK is untested.

[![GitHub tag](https://img.shields.io/github/tag/rust-iot/rust-radio-sx1231.svg)](https://github.com/rust-iot/rust-radio-sx1231)[![Build Status](https://github.com/rust-iot/rust-radio-sx1231/actions/workflows/run-test.yml/badge.svg)](https://github.com/rust-iot/rust-radio-sx1231/actions/workflows/run-test.yml)[![crates.io](https://img.shields.io/crates/v/radio-sx1231.svg)](https://crates.io/crates/radio-sx1231) [![Documentation](https://docs.rs/radio-sx1231/badge.svg)](https://docs.rs/radio-sx1231)

[Open Issues](https://github.com/rust-iot/rust-radio-sx1231/issues)

## Usage

Add to your project with `cargo add radio-sx1231`

Install the utility with `cargo install radio-sx1231`

## `no_std` Compability

The radio-sx1231 crate can be used as an interface library for the sx1231 radio on
embedded devices.


## Useful Resources
- [Datasheet](https://cdn-shop.adafruit.com/product-files/3076/sx1231.pdf)
