#![doc = include_str!("../README.md")]
//! ## Feature flags
#![doc = document_features::document_features!()]
// Make compiler and rust analyzer happy
#![allow(dead_code)]
#![allow(non_snake_case, non_upper_case_globals)]
// Enable std for espidf and test
#![cfg_attr(not(test), no_std)]

#[cfg(feature = "_esp_ble")]
pub use crate::ble::esp::initialize_esp_ble_keyboard_with_config_and_run;
#[cfg(feature = "_nrf_ble")]
pub use crate::ble::nrf::initialize_nrf_ble_keyboard_with_config_and_run;
#[cfg(not(feature = "rapid_debouncer"))]
use crate::debounce::default_bouncer::DefaultDebouncer;
#[cfg(feature = "rapid_debouncer")]
use crate::debounce::fast_debouncer::RapidDebouncer;
use crate::{
    keyboard::keyboard_task,
    light::{led_hid_task, LightService},
    via::vial_task,
};
use action::KeyAction;
use core::cell::RefCell;
use defmt::*;
use embassy_futures::select::{select, select4, Either, Either4};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Receiver, Sender},
};
use embassy_time::Timer;
use embassy_usb::driver::Driver;
pub use embedded_hal;
use embedded_hal::digital::{InputPin, OutputPin};
#[cfg(feature = "async_matrix")]
use embedded_hal_async::digital::Wait;
use embedded_storage::nor_flash::NorFlash;
use embedded_storage_async::nor_flash::NorFlash as AsyncNorFlash;
use futures::pin_mut;
use keyboard::{communication_task, Keyboard, KeyboardReportMessage};
use keymap::KeyMap;
use matrix::{Matrix, MatrixTrait, MuxMatrix};
pub use rmk_config as config;
use rmk_config::RmkConfig;
pub use rmk_macro as macros;
use storage::Storage;
use usb::KeyboardUsbDevice;
use via::process::VialService;

pub mod action;
#[cfg(feature = "_ble")]
pub mod ble;
mod debounce;
mod flash;
mod hid;
pub mod keyboard;
pub mod keyboard_macro;
pub mod keycode;
pub mod keymap;
pub mod layout_macro;
mod light;
mod matrix;
pub mod split;
mod storage;
mod usb;
mod via;

/// Initialize and run the keyboard service, with given keyboard usb config. This function never returns.
///
/// # Arguments
///
/// * `driver` - embassy usb driver instance
/// * `input_pins` - input gpio pins
/// * `output_pins` - output gpio pins
/// * `flash` - optional flash storage, which is used for storing keymap and keyboard configs
/// * `keymap` - default keymap definition
/// * `keyboard_config` - other configurations of the keyboard, check [RmkConfig] struct for details
pub async fn initialize_keyboard_and_run<
    F: NorFlash,
    D: Driver<'static>,
    #[cfg(feature = "async_matrix")] In: Wait + InputPin,
    #[cfg(not(feature = "async_matrix"))] In: InputPin,
    Out: OutputPin,
    const ROW: usize,
    const COL: usize,
    const NUM_LAYER: usize,
>(
    driver: D,
    #[cfg(feature = "col2row")] input_pins: [In; ROW],
    #[cfg(not(feature = "col2row"))] input_pins: [In; COL],
    #[cfg(feature = "col2row")] output_pins: [Out; COL],
    #[cfg(not(feature = "col2row"))] output_pins: [Out; ROW],
    flash: Option<F>,
    keymap: [[[KeyAction; COL]; ROW]; NUM_LAYER],
    keyboard_config: RmkConfig<'static, Out>,
) -> ! {
    // Wrap `embedded-storage` to `embedded-storage-async`
    let async_flash = flash.map(|f| embassy_embedded_hal::adapter::BlockingAsync::new(f));

    initialize_keyboard_and_run_async_flash(
        driver,
        input_pins,
        output_pins,
        async_flash,
        keymap,
        keyboard_config,
    )
    .await
}

/// Initialize and run the keyboard service, with given keyboard usb config. This function never returns.
///
/// # Arguments
///
/// * `driver` - embassy usb driver instance
/// * `input_pins` - input gpio pins
/// * `output_pins` - output gpio pins
/// * `flash` - optional **async** flash storage, which is used for storing keymap and keyboard configs
/// * `keymap` - default keymap definition
/// * `keyboard_config` - other configurations of the keyboard, check [RmkConfig] struct for details
pub async fn initialize_keyboard_and_run_async_flash<
    F: AsyncNorFlash,
    D: Driver<'static>,
    #[cfg(feature = "async_matrix")] In: Wait + InputPin,
    #[cfg(not(feature = "async_matrix"))] In: InputPin,
    Out: OutputPin,
    const ROW: usize,
    const COL: usize,
    const NUM_LAYER: usize,
>(
    driver: D,
    #[cfg(feature = "col2row")] input_pins: [In; ROW],
    #[cfg(not(feature = "col2row"))] input_pins: [In; COL],
    #[cfg(feature = "col2row")] output_pins: [Out; COL],
    #[cfg(not(feature = "col2row"))] output_pins: [Out; ROW],
    flash: Option<F>,
    default_keymap: [[[KeyAction; COL]; ROW]; NUM_LAYER],
    keyboard_config: RmkConfig<'static, Out>,
) -> ! {
    // Initialize storage and keymap
    let (mut storage, keymap) = match flash {
        Some(f) => {
            let mut s = Storage::new(f, &default_keymap, keyboard_config.storage_config).await;
            let keymap = RefCell::new(
                KeyMap::<ROW, COL, NUM_LAYER>::new_from_storage(default_keymap, Some(&mut s)).await,
            );
            (Some(s), keymap)
        }
        None => {
            let keymap = RefCell::new(
                KeyMap::<ROW, COL, NUM_LAYER>::new_from_storage::<F>(default_keymap, None).await,
            );
            (None, keymap)
        }
    };

    static keyboard_channel: Channel<CriticalSectionRawMutex, KeyboardReportMessage, 8> =
        Channel::new();
    let mut keyboard_report_sender = keyboard_channel.sender();
    let mut keyboard_report_receiver = keyboard_channel.receiver();

    // Create keyboard services and devices

    // Keyboard matrix, use COL2ROW by default
    #[cfg(all(feature = "col2row", feature = "rapid_debouncer"))]
    let matrix = Matrix::<_, _, RapidDebouncer<ROW, COL>, ROW, COL>::new(input_pins, output_pins);
    #[cfg(all(feature = "col2row", not(feature = "rapid_debouncer")))]
    let matrix = Matrix::<_, _, DefaultDebouncer<ROW, COL>, ROW, COL>::new(input_pins, output_pins);
    #[cfg(all(not(feature = "col2row"), feature = "rapid_debouncer"))]
    let matrix = Matrix::<_, _, RapidDebouncer<COL, ROW>, COL, ROW>::new(input_pins, output_pins);

    let (mut keyboard, mut usb_device, mut vial_service, mut light_service) = (
        Keyboard::new(matrix, &keymap),
        KeyboardUsbDevice::new(driver, keyboard_config.usb_config),
        VialService::new(&keymap, keyboard_config.vial_config),
        LightService::from_config(keyboard_config.light_config),
    );

    loop {
        // Run all tasks, if one of them fails, wait 1 second and then restart
        if let Some(ref mut s) = storage {
            run_usb_keyboard(
                &mut usb_device,
                &mut keyboard,
                s,
                &mut light_service,
                &mut vial_service,
                &mut keyboard_report_receiver,
                &mut keyboard_report_sender,
            )
            .await;
        } else {
            // Run 5 tasks: usb, keyboard, led, vial, communication
            let usb_fut = usb_device.device.run();
            let keyboard_fut = keyboard_task(&mut keyboard, &mut keyboard_report_sender);
            let communication_fut = communication_task(
                &mut keyboard_report_receiver,
                &mut usb_device.keyboard_hid_writer,
                &mut usb_device.other_hid_writer,
            );
            let led_fut = led_hid_task(&mut usb_device.keyboard_hid_reader, &mut light_service);
            let via_fut = vial_task(&mut usb_device.via_hid, &mut vial_service);
            pin_mut!(usb_fut);
            pin_mut!(keyboard_fut);
            pin_mut!(led_fut);
            pin_mut!(via_fut);
            pin_mut!(communication_fut);
            match select4(
                usb_fut,
                select(keyboard_fut, communication_fut),
                led_fut,
                via_fut,
            )
            .await
            {
                Either4::First(_) => {
                    error!("Usb task is died");
                }
                Either4::Second(_) => error!("Keyboard task is died"),
                Either4::Third(_) => error!("Led task is died"),
                Either4::Fourth(_) => error!("Via task is died"),
            }
        }

        warn!("Detected failure, restarting keyboard sevice after 1 second");
        Timer::after_secs(1).await;
    }
}

// Run usb keyboard task for once
pub(crate) async fn run_usb_keyboard<
    'a,
    'b,
    D: Driver<'a>,
    F: AsyncNorFlash,
    M: MatrixTrait,
    Out: OutputPin,
    const ROW: usize,
    const COL: usize,
    const NUM_LAYER: usize,
>(
    usb_device: &mut KeyboardUsbDevice<'a, D>,
    keyboard: &mut Keyboard<'b, M, ROW, COL, NUM_LAYER>,
    storage: &mut Storage<F>,
    light_service: &mut LightService<Out>,
    vial_service: &mut VialService<'b, ROW, COL, NUM_LAYER>,
    keyboard_report_receiver: &mut Receiver<'b, CriticalSectionRawMutex, KeyboardReportMessage, 8>,
    keyboard_report_sender: &mut Sender<'b, CriticalSectionRawMutex, KeyboardReportMessage, 8>,
) {
    let usb_fut = usb_device.device.run();
    let keyboard_fut = keyboard_task(keyboard, keyboard_report_sender);
    let communication_fut = communication_task(
        keyboard_report_receiver,
        &mut usb_device.keyboard_hid_writer,
        &mut usb_device.other_hid_writer,
    );
    let led_fut = led_hid_task(&mut usb_device.keyboard_hid_reader, light_service);
    let via_fut = vial_task(&mut usb_device.via_hid, vial_service);
    let storage_fut = storage.run::<ROW, COL, NUM_LAYER>();
    pin_mut!(usb_fut);
    pin_mut!(keyboard_fut);
    pin_mut!(led_fut);
    pin_mut!(via_fut);
    pin_mut!(storage_fut);
    pin_mut!(communication_fut);
    match select4(
        select(usb_fut, keyboard_fut),
        storage_fut,
        led_fut,
        select(via_fut, communication_fut),
    )
    .await
    {
        Either4::First(e) => match e {
            Either::First(_) => error!("Usb task is died"),
            Either::Second(_) => error!("Keyboard task is died"),
        },
        Either4::Second(_) => error!("Storage task is died"),
        Either4::Third(_) => error!("Led task is died"),
        Either4::Fourth(_) => error!("Via task is died"),
    }
}

pub(crate) fn reboot_keyboard() {
    warn!("Rebooting keyboard!");
    // For cortex-m:
    #[cfg(all(
        target_arch = "arm",
        target_os = "none",
        any(target_abi = "eabi", target_abi = "eabihf")
    ))]
    cortex_m::peripheral::SCB::sys_reset();
    // TODO: Implement reboot for other platforms
    // - RISCV
    // - ESP32
}

pub async fn mux_initialize_keyboard_and_run_async_flash<
    F: AsyncNorFlash,
    D: Driver<'static>,
    In: InputPin,
    Out: OutputPin,
    const NUM_LAYER: usize,
    const IDX_PIN_NUM: usize,
    const INPUT_PIN_NUM: usize,
    const OUTPUTS_PER_SIDE: usize,
    const TOTAL_OUTPUTS: usize,
>(
    driver: D,
    input_pins: [In; INPUT_PIN_NUM],
    output_pins: [Out; IDX_PIN_NUM], // We don't output directly to columns so outputs to IDX pins
    flash: Option<F>,
    default_keymap: [[[KeyAction; TOTAL_OUTPUTS]; INPUT_PIN_NUM]; NUM_LAYER],
    keyboard_config: RmkConfig<'static, Out>,
    side_pin: Out,
) -> ! {
    // Initialize storage and keymap
    let (mut storage, keymap) = match flash {
        Some(f) => {
            let mut s = Storage::new(f, &default_keymap, keyboard_config.storage_config).await;
            let keymap = RefCell::new(
                KeyMap::<INPUT_PIN_NUM, TOTAL_OUTPUTS, NUM_LAYER>::new_from_storage(
                    default_keymap,
                    Some(&mut s),
                )
                .await,
            );
            (Some(s), keymap)
        }
        None => {
            let keymap = RefCell::new(
                KeyMap::<INPUT_PIN_NUM, TOTAL_OUTPUTS, NUM_LAYER>::new_from_storage::<F>(
                    default_keymap,
                    None,
                )
                .await,
            );
            (None, keymap)
        }
    };

    static keyboard_channel: Channel<CriticalSectionRawMutex, KeyboardReportMessage, 8> =
        Channel::new();
    let mut keyboard_report_sender = keyboard_channel.sender();
    let mut keyboard_report_receiver = keyboard_channel.receiver();

    let matrix = MuxMatrix::<
        _,
        _,
        _,
        RapidDebouncer<INPUT_PIN_NUM, TOTAL_OUTPUTS>,
        IDX_PIN_NUM,
        INPUT_PIN_NUM,
        OUTPUTS_PER_SIDE,
        TOTAL_OUTPUTS,
    >::new(input_pins, output_pins, side_pin);

    let (mut keyboard, mut usb_device, mut vial_service, mut light_service) = (
        Keyboard::new(matrix, &keymap),
        KeyboardUsbDevice::new(driver, keyboard_config.usb_config),
        VialService::new(&keymap, keyboard_config.vial_config),
        LightService::from_config(keyboard_config.light_config),
    );

    loop {
        // Run all tasks, if one of them fails, wait 1 second and then restart
        if let Some(ref mut s) = storage {
            run_usb_keyboard(
                &mut usb_device,
                &mut keyboard,
                s,
                &mut light_service,
                &mut vial_service,
                &mut keyboard_report_receiver,
                &mut keyboard_report_sender,
            )
            .await;
        } else {
            // Run 5 tasks: usb, keyboard, led, vial, communication
            let usb_fut = usb_device.device.run();
            let keyboard_fut = keyboard_task(&mut keyboard, &mut keyboard_report_sender);
            let communication_fut = communication_task(
                &mut keyboard_report_receiver,
                &mut usb_device.keyboard_hid_writer,
                &mut usb_device.other_hid_writer,
            );
            let led_fut = led_hid_task(&mut usb_device.keyboard_hid_reader, &mut light_service);
            let via_fut = vial_task(&mut usb_device.via_hid, &mut vial_service);

            pin_mut!(usb_fut);
            pin_mut!(keyboard_fut);
            pin_mut!(led_fut);
            pin_mut!(via_fut);
            pin_mut!(communication_fut);

            #[cfg(feature = "usb_log")]
            let log_fut = usb_device.logger.run();
            #[cfg(feature = "usb_log")]
            pin_mut!(log_fut);

            match select4(
                usb_fut,
                select(keyboard_fut, communication_fut),
                #[cfg(not(feature = "usb_log"))]
                led_fut,
                #[cfg(feature = "usb_log")]
                select(led_fut, log_fut),
                via_fut,
            )
            .await
            {
                Either4::First(_) => {
                    error!("Usb task is died");
                }
                Either4::Second(_) => error!("Keyboard task is died"),
                Either4::Third(_) => error!("Led task is died"),
                Either4::Fourth(_) => error!("Via task is died"),
            }
        }

        warn!("Detected failure, restarting keyboard sevice after 1 second");
        Timer::after_secs(1).await;
    }
}
