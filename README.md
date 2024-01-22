# RMK

[![Crates.io](https://img.shields.io/crates/v/rmk)](https://crates.io/crates/rmk)
[![Docs](https://img.shields.io/docsrs/rmk)](https://docs.rs/rmk/latest/rmk/)
[![Build](https://github.com/haobogu/rmk/actions/workflows/build.yml/badge.svg)](https://github.com/HaoboGu/rmk/actions)

Keyboard firmware for cortex-m, with layer/dynamic keymap/[vial](https://get.vial.today) support, written in Rust.

## News

Rmk just released version 0.1.0, migrate to [Embassy](https://github.com/embassy-rs/embassy)! By migrating to Embassy, Rmk now has better async support, more supported MCUs and much easier usages than before. For examples using Embassy, check [`boards`](https://github.com/HaoboGu/rmk/tree/main/boards) folder!

## Prerequisites

This crate requires Rust 1.75 and up. `openocd`(stm32) or `probe-rs`(stm32/rp2040) is used for flashing & debugging.

## Usage

You can build your own keyboard firmware using RMK or try built-in firmware example for stm32h7 & rp2040.

### Build your own firmware
Example can be found at [`boards`](https://github.com/HaoboGu/rmk/blob/main/boards). The following is a simple
step-to-step instruction for creating your own firmware:

1. Create a rust embedded project, Add rmk to your project using `cargo add rmk`
2. Choose your target, use `rustup target add <your-target-name>` to install the
   target. [Here](https://docs.rust-embedded.org/book/intro/install.html) is the doc for target choosing. For example,
   rp2040 is Cortex-M0+, so its target is `thumbv6m-none-eabi`.
3. Create `.cargo/config.toml` in your project's root, specify your target here.
   See [`boards/stm32h7/.cargo/config.toml`](https://github.com/HaoboGu/rmk/blob/main/boards/stm32h7/.cargo/config.toml)
4. Create your own keymap file, see: [`boards/stm32h7/src/keymap.rs`](https://github.com/HaoboGu/rmk/blob/main/boards/stm32h7/src/keymap.rs)
5. Generate the vial config of your keymap, folloing vial's [porting guide](https://get.vial.today/docs/porting-to-via.html)
6. Create `main.rs`, create your `async fn main()`.
   See [`boards/stm32h7/src/main.rs`](https://github.com/HaoboGu/rmk/blob/main/boards/stm32h7/src/main.rs)

### Use built-in example

#### rp2040

1. Install [probe-rs](https://github.com/probe-rs/probe-rs)
   ```shell
   cargo install probe-rs --features cli
   ```
2. Build the firmware
   ```shell
   cd boards/rp2040
   cargo build
   ```
3. Flash it
   ```shell
   cd boards/rp2040
   cargo run
   ```

#### stm32h7

1. Install [openocd](https://github.com/openocd-org/openocd)
2. Build the firmware
   ```shell
   cd boards/stm32h7
   cargo build
   ```
3. Flash
   You can use both `probe-rs` and `openocd` to flash the firmware: 
   ```shell
   # Use openocd
   openocd -f openocd.cfg -c "program target/thumbv7em-none-eabihf/debug/rmk-stm32h7 preverify verify reset exit"
   
   # Use probe-rs
   cd boards/stm32h7
   cargo run
   ```

4. (Optional) Debug firmware using CMSIS-DAP

   Open the project using VSCode, choose `Cortex-Debug - stm32h7` debug profile, then press `F5`, the firmware will be automatically compiled and flashed. A debug session is started after flashing.
   Check [`.vscode/tasks.json`](https://github.com/HaoboGu/rmk/blob/main/.vscode/tasks.json) and [`.vscode/launch.json`](https://github.com/HaoboGu/rmk/blob/main/.vscode/launch.json) for details.

## Roadmap

A lot of todos at the list, any contributions are welcomed :)

- [x] support rp2040
- [x] basic keyboard functions
- [x] layer
- [x] system/media keys
- [x] vial support
- [x] eeprom
- [ ] keyboard macro
- [ ] wireless
- [ ] encoder
- [ ] RGB
- [ ] cli tools

## License
Rmk is licensed under either of

- Apache License, Version 2.0 (LICENSE-APACHE or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license (LICENSE-MIT or http://opensource.org/licenses/MIT)

at your option.
