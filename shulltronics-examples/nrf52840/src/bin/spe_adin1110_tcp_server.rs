//! Create a simple TCP echo server with the ADIN1110 

#![deny(clippy::pedantic)]
#![allow(clippy::doc_markdown)]
#![no_main]
#![no_std]
// Needed unitl https://github.com/rust-lang/rust/issues/63063 is stablised.
#![feature(type_alias_impl_trait)]
#![feature(associated_type_bounds)]
#![allow(clippy::missing_errors_doc)]

use core::sync::atomic::{AtomicI32, Ordering};

use defmt::{info, warn, error, unwrap};
use defmt_rtt as _; // global logger
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_futures::yield_now;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4};
use embassy_time::{Timer, Delay, Duration};
//use embedded_hal_async::spi::ExclusiveDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embedded_io::Write as bWrite;
use embedded_io_async::Write;

use embassy_nrf as hal;
use hal::bind_interrupts;
use hal::gpio::{Input, Pull, Output, OutputDrive, Level};
use hal::spim;
use hal::saadc;
use hal::rng;

use rand::RngCore;
use heapless::Vec;

//use display_interface_spi::SPIInterface;

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<hal::peripherals::RNG>;
    SPIM3 => spim::InterruptHandler<hal::peripherals::SPI3>;
    SAADC => saadc::InterruptHandler;
});

use static_cell::{StaticCell, make_static};
//use embassy_net_w5500::*;
use embassy_net_adin1110::*;
//use embassy_net_driver::Driver;

static VOLTAGE: AtomicI32 = AtomicI32::new(0);
type SpiBusShared = StaticCell<Mutex<ThreadModeRawMutex, spim::Spim<'static, hal::peripherals::SPI3>>>;
static SPI_BUS: SpiBusShared = StaticCell::new();

// Define some type alias for our hardware
type StatusLed   = Output<'static, hal::peripherals::P0_13>;
type BatteryAdc  = saadc::Saadc<'static, 1>;
type ADIN1110Cs  = Output<'static, hal::peripherals::P0_20>;
//type W5500Spi   = ExclusiveDevice<hal::spim::Spim<'static, hal::peripherals::SPI3>, W5500Cs, Delay>;
type ADIN1110Spi = SpiDevice<'static, ThreadModeRawMutex, spim::Spim<'static, hal::peripherals::SPI3>, ADIN1110Cs>;
type ADIN1110Int = Input<'static, hal::peripherals::P0_27>;
type ADIN1110Rst = Output<'static, hal::peripherals::P1_13>;

/// A task to blink the status led!
#[embassy_executor::task]
async fn heartbeat(mut led: StatusLed) -> ! {
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(100)).await;
        led.set_low();
        Timer::after(Duration::from_millis(900)).await;
    }
}

/// A task to measure the battery voltage!
#[embassy_executor::task]
async fn battery_voltage(mut saadc: BatteryAdc) -> ! {
    loop {
        let mut buf = [0; 1];
        saadc.sample(&mut buf).await;
        VOLTAGE.store(buf[0].into(), Ordering::Relaxed);
        Timer::after(Duration::from_millis(2000)).await;
    }
}

/// A task to process ethernet
#[embassy_executor::task]
async fn ethernet_task(runner: Runner<'static, ADIN1110Spi, ADIN1110Int, ADIN1110Rst>) -> ! {
    runner.run().await
}

/// A task to process network transactions
#[embassy_executor::task]
async fn net_task(stack: &'static Stack<Device<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Starting HTTP server example...");

    let dp = embassy_nrf::init(Default::default());

    // setup the status led and start the heartbeat
    let status_led = Output::new(dp.P0_13, Level::Low, OutputDrive::Standard);
    unwrap!(spawner.spawn(heartbeat(status_led)));

    // setup the ADC
    let mut voltage_pin = dp.P0_30;  // BATT_VIN/3 on schematic
    let config = saadc::Config::default();
    let channel_config = saadc::ChannelConfig::single_ended(&mut voltage_pin);
    let saadc = saadc::Saadc::new(dp.SAADC, Irqs, config, [channel_config]);
    unwrap!(spawner.spawn(battery_voltage(saadc)));

    // setup the shared SPI bus - TODO verify pins on MicroMod nRF52840 board
    let spi_sclk_pin = dp.P0_28;
    let spi_mosi_pin = dp.P0_31;
    let spi_miso_pin = dp.P0_02;
    let mut config   = spim::Config::default();
    config.frequency = spim::Frequency::M16;
    let spi_bus      = spim::Spim::new(dp.SPI3, Irqs, spi_sclk_pin, spi_miso_pin, spi_mosi_pin, config);
    let spi_bus      = Mutex::<ThreadModeRawMutex, _>::new(spi_bus);
    let spi_bus      = SPI_BUS.init(spi_bus);

    // setup the ADIN1110 SpiDevice; these pins correspond to the nRF52840 MicroMod processor
    // used with the MicroMod SPE Function Board.
    let adin1110_cs_pin  = dp.P0_20; // SPI_CS on schematic
    let adin1110_int_pin = dp.P0_27; // D0 label on schematic
    let adin1110_rst_pin = dp.P1_13; // GPIO2 on schematic
    let adin1110_ncs = Output::new(adin1110_cs_pin, Level::High, OutputDrive::Standard);
    let adin1110_nint = Input::new(adin1110_int_pin, Pull::Up);
    let adin1110_nrst = Output::new(adin1110_rst_pin, Level::High, OutputDrive::Standard);
    let adin1110_spi_dev = SpiDevice::new(spi_bus, adin1110_ncs);

    // create another SPI device
    // let ili9341_ndc = Output::new(dp.P0_05, Level::High, OutputDrive::Standard);
    // let ili9341_ncs = Output::new(dp.P0_06, Level::High, OutputDrive::Standard);
    // let ili9341_spi_dev = SpiDevice::new(spi_bus, ili9341_ncs);
    // let display_iface = SPIInterface::new(ili9341_spi_dev, ili9341_ndc);
    //let display = Ili9341::new(display_iface, ili9341_rst, Delay);

    // Create the ADIN1110 driver
    let mac_addr: [u8; 6] = [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff];
    let state = make_static!(embassy_net_adin1110::State::<8, 8>::new());
    let (device, runner) = embassy_net_adin1110::new(
        mac_addr,
        state,
        //ExclusiveDevice::new(w5500_spi, w5500_ncs, Delay),
        adin1110_spi_dev,
        adin1110_nint,
        adin1110_nrst,
        true,                   // whether or not to do a CRC check, make sure it matches SPI_CFG0 hw pin
    )
    .await;
    //defmt::info!("w5500 capabilities: {:?}", device.capabilities());
    unwrap!(spawner.spawn(ethernet_task(runner)));

    // Generate random seed
    let mut rng = rng::Rng::new(dp.RNG, Irqs);
    let seed = rng.next_u64();
    
    let ip_address: Ipv4Cidr = Ipv4Cidr::new(Ipv4Address([10, 11, 12, 13]), 24);
    let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: ip_address,
        gateway: None,
        dns_servers: Vec::new(),
    });

    // Init network stack
    let stack = &*make_static!(Stack::new(
        device,
        //embassy_net::Config::dhcpv4(Default::default()),
        net_config,
        make_static!(StackResources::<2>::new()),
        seed
    ));
    unwrap!(spawner.spawn(net_task(stack)));

    info!("waiting for DHCP config...");
    let cfg = wait_for_config(stack).await;
    let local_addr = cfg.address.address();
    info!("got an IP address: {:?}", local_addr);

    // let mut rx_buffer = [0; 4096];
    // let mut tx_buffer = [0; 4096];
    // let mut buf = [0; 4096];
    // loop {
    //     let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    //     socket.set_timeout(Some(Duration::from_secs(10)));

    //     info!("Listening on TCP:1234...");
    //     if let Err(e) = socket.accept(1234).await {
    //         warn!("accept error: {:?}", e);
    //         continue;
    //     }
    //     info!("Received connection from {:?}", socket.remote_endpoint());

    //     loop {
    //         let n = match socket.read(&mut buf).await {
    //             Ok(0) => {
    //                 warn!("read EOF");
    //                 break;
    //             }
    //             Ok(n) => n,
    //             Err(e) => {
    //                 warn!("{:?}", e);
    //                 break;
    //             }
    //         };
    //         info!("rxd {}", core::str::from_utf8(&buf[..n]).unwrap());

    //         if let Err(e) = socket.write_all(&buf[..n]).await {
    //             warn!("write error: {:?}", e);
    //             break;
    //         }
    //     }
    // }

    let http_port = 80;

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut mb_buf = [0; 4096];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        info!("Listening for TCP connections on {}:{}...", local_addr, http_port);
        if let Err(e) = socket.accept(http_port).await {
            defmt::error!("accept error: {:?}", e);
            continue;
        }

        loop {
            let n = match socket.read(&mut mb_buf).await {
                Ok(0) => {
                    info!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    error!("{:?}", e);
                    break;
                }
            };
            info!("rxd {}", core::str::from_utf8(&mb_buf[..n]).unwrap());

            if let Err(e) = socket.write_all(&mb_buf[..n]).await {
                error!("write error: {:?}", e);
                break;
            }

        }
    }
}

async fn wait_for_config(stack: &'static Stack<Device<'static>>) -> embassy_net::StaticConfigV4 {
    loop {
        if let Some(config) = stack.config_v4() {
            return config;
        }
        yield_now().await;
    }
}
