//! # i2c_hung_fix
//!
//! Attempts to unhang the I2C bus after an ungraceful reset using traits from the embedded-hal.
//!
//! One of the shortcoming of the I2C standard is that if the master is power cycled or
//! crashes in the middle of a read from a slave, the slave will keep the data line held
//! low because it is waiting to finish sending data to the master.
//!
//! If the data line is held low, initializing I2C on the master side will fail because
//! the slave won't repsond to the startup procedure. This is typically called a hung bus.
//!
//! I implemented this because my I2C bus was getting hung regularly when reprogramming my
//! device during development & debugging but it's a well known issue people have to mitigate
//! in production hardware.
//!
//! Analog Devices have an app note describing the issue and two solutions:
//! [Implementing an I2C Reset](https://www.analog.com/media/en/technical-documentation/application-notes/54305147357414AN686_0.pdf)
//!
//! The hardware solution of allowing the master to power cycle the slaves is the best if
//! the hardware can be added to the design but the software solution also works in many
//! cases.
//!
//! This crate implements the sofware solution to a hung bus. It is simply clocking the
//! SCL clock line until the slave releases the SDA data line.
//!
//! It is reasonable to run the hung fix on every boot up; it will exit after a single
//! period of the I2C clock if the bus isn't hung.
//!
//! # Example
//!
//! This example is using embassy-stm32
//! ```
//! let mut scl = Output::new(unsafe { PB10::steal() }, Level::Low, Speed::Low);
//! let sda = Input::new(unsafe { PB11::steal() }, Pull::None);
//!
//!
//! try_unhang_i2c(&mut scl, &sda, &mut delay, 4_000_000, RECOMMENDED_MAX_CLOCK_CYCLES)
//!     .expect("I2C bus is still hung after try_unhang_i2c");
//! ```

#![no_std]
#![deny(warnings, missing_docs)]

use embassy_stm32::gpio::{Input, Output};
use embassy_time::{Duration, Timer};

/// The possible sucessful results to applying the hung bus fix
#[derive(Debug, Clone, Copy)]
pub enum Sucess {
    /// The bus wasn't hung in the first place
    BusNotHung,
    /// The bus was hung but we unhung it
    FixedHungBus,
}

/// Errors we can return
#[derive(Debug, Clone, Copy)]
pub enum Error {
    /// We clocked the bus for the max allowed clock cycles and it was still hung
    StillHung,
}

/// The recommended default value for the max_clock_cycles parameter
///
/// According to the app note listed above, 9 clock toggles should be sufficient for all
/// stuck cases but experimentally I've found my device would sometimes take up to 12.
/// 15 should be sufficient and isn't so high that it'll introduce much delay if it fails
/// to resolve the issue.
pub const RECOMMENDED_MAX_CLOCK_CYCLES: u8 = 15;

/// A fallback I2C frequency that can be used if the caller doesn't know what
/// frequency the I2C bus should be running at (buried in a library perhaps)
pub const FALLBACK_I2C_FREQUENCY: u32 = 100_000;

/// This function applies the software based hung bus fix described the the lib doc
///
/// # Arguments
///
/// * `scl` - the clock pin.
/// * `sda` - the data pin.
/// * `i2c_frequency` - the frequency the I2C bus should be clocked at in Hertz. Can use
///       [FALLBACK_I2C_FREQUENCY](constant.FALLBACK_I2C_FREQUENCY.html) if the frequency is unknown but
///       getting the right frequency for the slave is preferable
/// * `max_clock_cycles` - the maxiumum number of times the clock will be cycled high->low->high before giving
///       up and returning [Error::StillHung](enum.Error.html#variant.StillHung).
///       [RECOMMENDED_MAX_CLOCK_CYCLES](constant.RECOMMENDED_MAX_CLOCK_CYCLES.html) is a reasonable default
///       value but this can be increased for misbehaving slaves devices
pub async fn try_unhang_i2c(
    scl: &mut Output<'static>,
    sda: &Input<'static>,
    i2c_frequency: u32,
    max_clock_cycles: u8,
) -> Result<Sucess, Error> {
    const US_PER_SECOND: u32 = 1_000_000;
    let delay_us = US_PER_SECOND / (i2c_frequency * 2);
    let wait_one_period = || Timer::after(Duration::from_micros(delay_us.into()));

    // The i2c clock idles high
    scl.set_high();
    wait_one_period().await;

    let mut was_hung = false;

    for _ in 0..max_clock_cycles {
        // Check if SDA is being held low by a slave
        if sda.is_high() {
            // No, bus should be ok to go
            return match was_hung {
                true => Ok(Sucess::FixedHungBus),
                false => Ok(Sucess::BusNotHung),
            };
        }
        // A slave is holding SDA low, which means the bus is hung
        was_hung = true;

        // Set the clock low to clock out a bit if the slave is waiting
        scl.set_low();
        wait_one_period().await;

        // Set the clock high again, slave should release SDA if it's done
        scl.set_high();
        wait_one_period().await;
    }

    Err(Error::StillHung)
}
