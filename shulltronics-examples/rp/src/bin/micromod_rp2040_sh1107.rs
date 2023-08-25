//! This example implements a TCP echo server on port 1234 and using DHCP.
//! Send it some data, you should see it echoed back and printed in the console.
//!
//! Example written for Adafruit Feather RP2040 and Particle Ethernet FeatherWing.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use {defmt_rtt as _, panic_probe as _};
use defmt::*;

use embassy_executor::Spawner;
use embassy_rp::peripherals::*;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, Config};
use embassy_time::{Timer, Duration};

// Imports for the OLED display
use sh1107::{
    prelude::*,
    mode::GraphicsMode,
    interface::i2c::I2cInterface,
    Builder
};
// A type alias to make things easier to read
// type SclPin = Output<'static, PIN_14>;
// type SdaPin = Output<'static, PIN_15>;
type OledDisplay = GraphicsMode<
    I2cInterface<embassy_rp::i2c::I2c<'static, I2C0, embassy_rp::i2c::Blocking>>
>;

type StatusLed  = Output<'static, PIN_25>;
#[embassy_executor::task]
async fn blink(mut led: StatusLed) -> ! {
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(100)).await;
        led.set_low();
        Timer::after(Duration::from_millis(900)).await;

    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = embassy_rp::init(Default::default());
    let mut led = Output::new(peripherals.PIN_25, Level::Low);

    let sda = peripherals.PIN_4;
    let scl = peripherals.PIN_5;

    info!("set up i2c ");
    let mut i2c = i2c::I2c::new_blocking(peripherals.I2C0, scl, sda, Config::default());

    // Setup the OLED display
    let display_size = DisplaySize::Display64x128;
    let display_rot  = DisplayRotation::Rotate270;
    let mut display: GraphicsMode<_> = Builder::new()
        .with_size(display_size)
        .with_rotation(display_rot)
        .connect_i2c(i2c)
        .into();
    display.init().unwrap();
    use embedded_graphics::{
        // primitives::{Rectangle, PrimitiveStyle},
        mono_font::{ascii::FONT_6X9, MonoTextStyle},
        pixelcolor::BinaryColor,
        prelude::*,
        text::Text,
    };
    let style = MonoTextStyle::new(&FONT_6X9, BinaryColor::On);
    let text = Text::new("Embassy testing", Point::new(10, 10), style);
    text.draw(&mut display).unwrap();
    display.flush().unwrap();

    unwrap!(spawner.spawn(blink(led)));
}
