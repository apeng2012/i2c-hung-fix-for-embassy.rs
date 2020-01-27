# i2c_hung_fix
 
Attempts to unhang the I2C bus after an ungraceful reset using traits from the embedded-hal.

[![Crate](https://img.shields.io/crates/v/i2c_hung_fix.svg)](https://crates.io/crates/i2c_hung_fix)
[![Documentation](https://docs.rs/i2c_hung_fix/badge.svg)](https://docs.rs/i2c_hung_fix)

One of the shortcoming of the I2C standard is that if the master is power cycled or
crashes in the middle of a read from a slave, the slave will keep the data line held
low because it is waiting to finish sending data to the master.

If the data line is held low, initializing I2C on the master side will fail because
the slave won't repsond to the startup procedure. This is typically called a hung bus.

I implemented this because my I2C bus was getting hung regularly when reprogramming my
device during development & debugging but it's a well known issue people have to mitigate 
in production hardware.
 
Analog Devices have an app note describing the issue and two solutions:
[Implementing an I2C Reset](https://www.analog.com/media/en/technical-documentation/application-notes/54305147357414AN686_0.pdf)

The hardware solution of allowing the master to power cycle the slaves is the best if
the hardware can be added to the design but the software solution also works in many
cases. 

This crate implements the sofware solution to a hung bus. It is simply clocking the 
SCL clock line until the slave releases the SDA data line.
 
It is reasonable to run the hung fix on every boot up; it will exit after a single
period of the I2C clock if the bus isn't hung.
 
## Example
 
This example is using stm32f1xx-hal 
```
let mut rcc = device.RCC.constrain();
let mut flash = device.FLASH.constrain();
let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
let mut scl = gpiob.pb10.into_push_pull_ouput();
let sda = gpiob.pb11;

let clocks = rcc.cfgr.freeze(&mut flash.acr);
let mut delay = Delay::new(core.SYST, clocks);

try_unhang_i2c(&mut scl, &sda, &mut delay, 4_000_000, RECOMMENDED_MAX_CLOCK_CYCLES)
    .expect("I2C bus is still hung after try_unhang_i2c");
```

## License

Free and open source software distributed under the terms of both the [MIT License][lm] and the [Apache License 2.0][la].

[lm]: LICENSE-MIT
[la]: LICENSE-APACHE