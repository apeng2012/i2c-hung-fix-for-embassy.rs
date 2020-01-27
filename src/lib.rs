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
//! This example is using stm32f1xx-hal 
//! ```
//! let mut rcc = device.RCC.constrain();
//! let mut flash = device.FLASH.constrain();
//! let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
//! let mut scl = gpiob.pb10.into_push_pull_ouput();
//! let sda = gpiob.pb11;
//! 
//! let clocks = rcc.cfgr.freeze(&mut flash.acr);
//! let mut delay = Delay::new(core.SYST, clocks);
//! 
//! try_unhang_i2c(&mut scl, &sda, &mut delay, 4_000_000, RECOMMENDED_MAX_CLOCK_CYCLES)
//!     .expect("I2C bus is still hung after try_unhang_i2c");
//! ```

#![no_std] 
#![deny(warnings, missing_docs)]

use embedded_hal::{
    digital::v2::{ OutputPin, InputPin },
    blocking::delay::DelayUs,
};

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
pub enum Error<E> {
    /// We clocked the bus for the max allowed clock cycles and it was still hung
    StillHung,
    /// An error occured from a call to a fn on OutputPin or InputPin
    HalError(E),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::HalError(e)
    }
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
/// * `scl` - the clock pin. Must implement [OutputPin](../embedded_hal/digital/v2/trait.OutputPin.html) trait
/// * `sda` - the data pin. Must implement [InputPin](../embedded_hal/digital/v2/trait.InputPin.html) trait
/// * `delay` - mutable reference to a struct that can perform blocking nano second delays. Must implement 
///       [DelayUs](../embedded_hal/blocking/delay/trait.DelayUs.html) trait
/// * `i2c_frequency` - the frequency the I2C bus should be clocked at in Hertz. Can use 
///       [FALLBACK_I2C_FREQUENCY](constant.FALLBACK_I2C_FREQUENCY.html) if the frequency is unknown but
///       getting the right frequency for the slave is preferable
/// * `max_clock_cycles` - the maxiumum number of times the clock will be cycled high->low->high before giving
///       up and returning [Error::StillHung](enum.Error.html#variant.StillHung). 
///       [RECOMMENDED_MAX_CLOCK_CYCLES](constant.RECOMMENDED_MAX_CLOCK_CYCLES.html) is a reasonable default 
///       value but this can be increased for misbehaving slaves devices
pub fn try_unhang_i2c<C, A, D, E>(scl: &mut C, sda: &A, delay: &mut D, i2c_frequency: u32, max_clock_cycles: u8) -> Result<Sucess, Error<E>> 
where
    C: OutputPin<Error = E>, 
    A: InputPin<Error = E>,
    D: DelayUs<u32>,
{
    const US_PER_SECOND: u32 = 1_000_000;
    let delay_us = US_PER_SECOND / (i2c_frequency * 2);
    let mut wait_one_period = || delay.delay_us(delay_us);

    // The i2c clock idles high
    scl.set_high()?;
    wait_one_period();

    let mut was_hung = false;

    for _ in 0..max_clock_cycles {
        // Check if SDA is being held low by a slave
        if sda.is_high()? {
            // No, bus should be ok to go
            return match was_hung {
                true => Ok(Sucess::FixedHungBus),
                false => Ok(Sucess::BusNotHung),
            };
        }
        // A slave is holding SDA low, which means the bus is hung
        was_hung = true;

        // Set the clock low to clock out a bit if the slave is waiting
        scl.set_low()?;
        wait_one_period();

        // Set the clock high again, slave should release SDA if it's done
        scl.set_high()?;
        wait_one_period();
    }

    Err(Error::StillHung)
}

/// This conveneince trait is for providing try_unhang_i2c on known I2C pins
/// in device hal implementation crates.
/// 
/// It is implemented here for tuples of (OutputPin, InputPin) so consumers can do:
/// ```
/// let mut pins = (gpiox.px1.into_push_pull_ouput(...), gpiox.px2.into_floating_input(...));
/// pins.try_unhang_i2c(delay)?;
/// let pins = (pins.0.into_alternate_open_drain(), pins.1.into_alternate_open_drain());
/// ```
pub trait HangFixPins<C, A, D, E>
where
    C: OutputPin<Error = E>, 
    A: InputPin<Error = E>,
    D: DelayUs<u32> 
{
    /// Runs the unhang fix with default settings 
    fn try_unhang_i2c(&mut self, delay: &mut D) -> Result<Sucess, Error<E>>;
}

impl<C, A, D, E> HangFixPins<C, A, D, E> for (C, A)
where
    C: OutputPin<Error = E>, 
    A: InputPin<Error = E>,
    D: DelayUs<u32> 
{
    fn try_unhang_i2c(&mut self, delay: &mut D) -> Result<Sucess, Error<E>> {
        try_unhang_i2c(&mut self.0, &self.1, delay, FALLBACK_I2C_FREQUENCY, RECOMMENDED_MAX_CLOCK_CYCLES)
    }
}
/*

/// This trait provides the functionality for attempting to clear a hung bus cause by the master 
/// not correctly ending the transaction. If the master stops driving the clock line while the slave
/// is clocking out data the slave will keep holding the SDA line low which will cause future I2C
/// actions to fail.
/// Occurs regularly if you are flashing the hardware while it's using I2C but could also occur 
/// after real resets on the final device. 
/// Fully detailed in (Implementing an I2C Reset)[https://www.analog.com/media/en/technical-documentation/application-notes/54305147357414AN686_0.pdf]
/// This essentially implements "Solution 1: Clocking Through the Problem"
/// The code is structured so that this fix can be run on the SDA & SCL pins before the I2C peripheral
/// is initialized: if there's no problem it'll just be a single read from the SDA pin.
pub trait HungFix<P: OutputPin<Error=Infallible>, Q: InputPin<Error=Infallible>> {
    fn scl(&mut self) -> &mut P;
    fn sda(&self) -> &Q;
    fn sda_state(&self) -> State {
        // Error type is constrained to be Infallible
        if self.sda().is_high().unwrap() {
            State::High
        } else {
            State::Low
        }
    }
    fn set_scl(&mut self, state: State) {
        match state {
            State::High => self.scl().set_high(),
            State::Low => self.scl().set_low(),
        }
        // Error type is constrained to be Infallible
        .unwrap()
    }
    /// Checks the SDA line for low state and toggles SCL to attempt to resolve a hung bus
    fn try_hung_fix(&mut self, delay: &mut Delay, mode: Mode) -> Result<(), Error> {
        // According to app note listed above 9 clock toggles should be sufficient for all stuck cases but experimentally
        // I found my device would sometimes take up to 12. Either way 30 should be sufficient and isn't so high that it'll 
        // introduce much delay if it fails to resolve the issue
        const TOGGLE_LIMIT: usize = 30;

        let delay_us = (1_000_000 / mode.get_frequency().0) / 2;

        // The clock idles high
        self.set_scl(State::High);
        delay.delay_us(delay_us);
        
        for _ in 0..TOGGLE_LIMIT {
            self.set_scl(State::Low);
            delay.delay_us(delay_us);
            
            self.set_scl(State::High);
            delay.delay_us(delay_us);
            
            if self.sda_state() == State::High {
                return Ok(());
            }
            debug!("SDA is stuck low");
        }

        Err(Error::HungBus)
    }
}

impl<MODE> HungFix<PB10<Output<PushPull>>, PB11<Input<MODE>>> for (PB10<Output<PushPull>>, PB11<Input<MODE>>) {
    fn scl(&mut self) -> &mut PB10<Output<PushPull>> { &mut self.0 }
    fn sda(&self) -> &PB11<Input<MODE>> { &self.1 }
}*/