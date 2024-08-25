#![no_std]
#![no_main]

mod blinky;
use blinky::BlinkPeripherals;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::dma::Channel;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::spi::{Config, Spi};
use embassy_time::Delay;
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::lorawan_radio::LorawanRadio;
use lora_phy::sx126x::{self, Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::LoRa;
use lorawan_device::async_device::{region, Device, EmbassyTimer, JoinMode};
use lorawan_device::default_crypto::DefaultFactory as Crypto;
use lorawan_device::{AppEui, AppKey, DevEui};
use {defmt_rtt as _, panic_probe as _};
use {defmt_rtt as _, panic_probe as _};

// warning: set these appropriately for the region
const LORAWAN_REGION: region::Region = region::Region::EU868;
const MAX_TX_POWER: u8 = 14;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialise Peripherals
    let p = embassy_rp::init(Default::default());

    // Initialize the peripherals for the status blinky
    {
        #[cfg(feature = "pico_w")]
        let p = BlinkPeripherals {
            pwr: p.PIN_23,
            cs: p.PIN_25,
            dio: p.PIN_24,
            clk: p.PIN_29,
            dma_ch: p.DMA_CH2.degrade(),
            pio: p.PIO0,
        };

        #[cfg(feature = "pico_non_w")]
        let p = BlinkPeripherals { led: p.PIN_25 };

        blinky::init(Duration::from_millis(100), spawner, p).await;
    }

    {
        // Initialize the LoRa device
        // I'm not able to move this to a separate file bcause of waaay to many generics
        let device = {
            let nss = Output::new(p.PIN_3.degrade(), Level::High);
            let reset = Output::new(p.PIN_15.degrade(), Level::High);
            let dio1 = Input::new(p.PIN_20.degrade(), Pull::None);
            let busy = Input::new(p.PIN_2.degrade(), Pull::None);

            let spi = Spi::new(
                p.SPI1,
                p.PIN_10,
                p.PIN_11,
                p.PIN_12,
                p.DMA_CH0,
                p.DMA_CH1,
                Config::default(),
            );
            let spi = ExclusiveDevice::new(spi, nss, Delay);

            let config = sx126x::Config {
                chip: Sx1262,
                tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
                use_dcdc: true,
                rx_boost: true,
            };

            let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();
            let lora = LoRa::new(Sx126x::new(spi, iv, config), true, Delay)
                .await
                .unwrap();

            let radio: LorawanRadio<_, _, MAX_TX_POWER> = lora.into();
            let region: region::Configuration = region::Configuration::new(LORAWAN_REGION);
            let mut device: Device<_, Crypto, _, _> = Device::new(
                region,
                radio,
                EmbassyTimer::new(),
                embassy_rp::clocks::RoscRng,
            );
            device
        };

        //TODO: ACtually join
    }

    // Loop
    loop {
        blinky::PERIOD.signal(Duration::from_millis(100));
        Timer::after(Duration::from_millis(1000)).await;

        blinky::PERIOD.signal(Duration::from_millis(20));
        Timer::after(Duration::from_millis(1000)).await;

        blinky::PERIOD.signal(Duration::from_millis(1000));
        Timer::after(Duration::from_millis(1500)).await;
    }
}
