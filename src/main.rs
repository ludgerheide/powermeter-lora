#![no_std]
#![no_main]

mod blinky;
mod iec62056;
use core::panic;
use core::sync::atomic::Ordering;

use bincode::{config, encode_into_slice, Decode, Encode};
use blinky::BlinkPeripherals;
use const_hex::decode_to_array;
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_rp::adc::Channel as AdcChannel;
use embassy_rp::adc::{Adc, Async};
use embassy_rp::bind_interrupts;
use embassy_rp::dma::Channel as DmaChannel;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::peripherals::UART0;
use embassy_rp::spi::{Config, Spi};
use embassy_rp::uart::BufferedInterruptHandler;
use embassy_rp_flash_struct::FlashStorage;
use embassy_time::{with_timeout, Duration};
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use iec62056::EnergyMeter;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::lorawan_radio::LorawanRadio;
use lora_phy::sx126x::{self, Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::LoRa;
use lorawan_device::async_device::{
    radio, region, Device, EmbassyTimer, JoinMode, JoinResponse, SendResponse, Timings,
};
use lorawan_device::default_crypto::DefaultFactory as Crypto;
use lorawan_device::{AppEui, AppKey, CryptoFactory, DevEui, RngCore};
use portable_atomic::AtomicU64;
use {defmt_rtt as _, panic_probe as _};

// warning: set these appropriately for the region
const LORAWAN_REGION: region::Region = region::Region::EU868;
const MAX_TX_POWER: u8 = 14;

// The durations and timeouts used within this struct are all centrally defined here
const METER_TIMEOUT: Duration = Duration::from_secs(10); // How long to wait for the energy meter's serial port to respond
const MEASUREMENT_TRANSMIT_INTERVAL: Duration = Duration::from_secs(30); // How long to sleep between sending messages
const RANDOM_SLEEP_VARIATION: Duration = Duration::from_secs(1); // The MEASUREMENT_TRANSMIT_INTERVAL is randomly appended this value. This reduces simultaneous transmissions

// This is the amount of channels used for listening on the S0 bus. 6 is the hightest value we are expecting in our use case
const S0_CHANNEL_COUNT: usize = 6;
static S0_COUNTERS: [AtomicU64; S0_CHANNEL_COUNT] = [const { AtomicU64::new(0) }; S0_CHANNEL_COUNT];
const S0_IMP_PER_KWH: [f32; S0_CHANNEL_COUNT] = [800.0; S0_CHANNEL_COUNT];

// We save the counter values to flash, so continue counting up over device resets
#[derive(Default, Encode, Decode)]
pub struct CounterValues {
    counts: [u64; S0_CHANNEL_COUNT],
}

// What will get transmitted over the air
#[derive(Encode)]
pub struct Transmission {
    flash_wear_fraction: f32, // 0 to 1, with 0 being new, 1 being totally worn
    temperature: f32,         //In degrees celsius

    main_meter_kwh: f32, // From the IEC62056 connection
    counter_0_kwh: f32,  // From the S0 counters
    counter_1_kwh: f32,  // From the S0 counters
    counter_2_kwh: f32,  // From the S0 counters
    counter_3_kwh: f32,  // From the S0 counters
    counter_4_kwh: f32,  // From the S0 counters
    counter_5_kwh: f32,  // From the S0 counters
}

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialise Peripherals
    let p = embassy_rp::init(Default::default());

    // ---------------- Start the tasks that update the values for the counters whenever they are updated ---------------
    {
        //spawner.spawn(blink_task(control, initial_period)).unwrap();
        spawner
            .spawn(counter_task(Input::new(p.PIN_16, Pull::Down), 0))
            .unwrap();
        spawner
            .spawn(counter_task(Input::new(p.PIN_17, Pull::Down), 1))
            .unwrap();
        spawner
            .spawn(counter_task(Input::new(p.PIN_18, Pull::Down), 2))
            .unwrap();
        spawner
            .spawn(counter_task(Input::new(p.PIN_19, Pull::Down), 3))
            .unwrap();
        spawner
            .spawn(counter_task(Input::new(p.PIN_21, Pull::Down), 4))
            .unwrap();
        spawner
            .spawn(counter_task(Input::new(p.PIN_22, Pull::Down), 5))
            .unwrap();
    }

    // ---------------- Initialize the Status blinky --------------------
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

    //---------------------Initialize the ADC to read temperature and battery voltage-------------
    let mut adc = Adc::new(p.ADC, Irqs, embassy_rp::adc::Config::default());
    let mut temp_chan = AdcChannel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    // ---------------- Initialize the LoRa Radio -----------------
    // I'm not able to move this to a separate file bcause of waaay to many generics
    let mut device = {
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
        let device: Device<_, Crypto, _, _> = Device::new(
            region,
            radio,
            EmbassyTimer::new(),
            embassy_rp::clocks::RoscRng,
        );
        device
    };
    join_network(&mut device).await;

    // Load in the saved counter values form flash, if they exist
    let mut persistent_storage: FlashStorage<CounterValues> =
        FlashStorage::new(p.FLASH, p.DMA_CH3.degrade());
    {
        let current_value = persistent_storage.read().await;
        for (i, counter) in S0_COUNTERS.iter().enumerate().take(S0_CHANNEL_COUNT) {
            let current_value_from_flash = current_value.counts[i];
            counter.fetch_add(current_value_from_flash, Ordering::Relaxed);
        }
    }

    // Initialize the UART energy meter reader
    let mut meter_connection = EnergyMeter::new(p.UART0, Irqs, p.PIN_1, p.PIN_0);

    // Loop
    loop {
        {
            //--------------------------------- Acquire Sensor Data -------------------------------------
            blinky::PERIOD.signal(Duration::from_millis(500));

            // Start the acquisition process for battery data (it runs in the background)
            let analog_data_future = temperature(&mut temp_chan, &mut adc);
            let meter_energy = match with_timeout(METER_TIMEOUT, meter_connection.get_data()).await
            {
                Err(_) => {
                    warn!("Timeout reading from energy meter!");
                    None
                }
                Ok(result) => Some(result.total_in),
            };
            let temperature = analog_data_future.await;

            let mut counter_kwh: [f32; S0_CHANNEL_COUNT] = [0.0; S0_CHANNEL_COUNT];
            for (i, counter) in S0_COUNTERS.iter().enumerate().take(S0_CHANNEL_COUNT) {
                let current_counter_value = counter.load(Ordering::Relaxed);
                let current_kwh_value = current_counter_value as f32 / S0_IMP_PER_KWH[i];
                counter_kwh[i] = current_kwh_value;
            }

            //--------------------------------- Prepare and transmit -------------------------------------
            blinky::PERIOD.signal(Duration::from_millis(50));
            let to_transmit = Transmission {
                flash_wear_fraction: persistent_storage.exhaustion(),
                temperature,

                main_meter_kwh: match meter_energy {
                    None => f32::NAN,
                    Some(val) => val,
                },
                counter_0_kwh: counter_kwh[0],
                counter_1_kwh: counter_kwh[1],
                counter_2_kwh: counter_kwh[2],
                counter_3_kwh: counter_kwh[3],
                counter_4_kwh: counter_kwh[4],
                counter_5_kwh: counter_kwh[5],
            };
            if size_of::<Transmission>() > 49 {
                panic!("Maximum transmission size for DR0 exceeded!");
            }
            let mut transmission_buf = [0u8; size_of::<Transmission>()];
            let size =
                encode_into_slice(to_transmit, &mut transmission_buf, config::standard()).unwrap();
            if size != size_of::<Transmission>() {
                panic!("Encoding did something unexpected!");
            }

            let resp = device.send(&transmission_buf, 1, false).await;
            match resp {
                Ok(send_resp) => {
                    info!("Sending okay: {:?}", send_resp);
                    match send_resp {
                        SendResponse::DownlinkReceived(_) => {
                            // Handle downlink requests
                            // We have received a donlink, but it does not necessarily contain information
                            let downlink = device.take_downlink();
                            match downlink {
                                None => info!("Downlink empty!"),
                                Some(data) => {
                                    // We can update the counter values using the downlink.
                                    // FPORT-1 is the counter to update
                                    // The payload should be a 8-byte value
                                    let counter_to_update = (data.fport - 1) as usize;
                                    if counter_to_update > S0_CHANNEL_COUNT {
                                        error!("Invalid FPORT {:?}", counter_to_update);
                                    } else {
                                        // The payload should be an 8-byte value
                                        if data.data.len() != 8 {
                                            error!("Invalid data len {:?}", data.data.len());
                                        } else {
                                            let buf = data.data.into_array().unwrap();
                                            let new_counter_value = u64::from_le_bytes(buf);
                                            S0_COUNTERS[counter_to_update]
                                                .store(new_counter_value, Ordering::Relaxed);
                                        }
                                    }
                                }
                            }
                        }
                        // If our session expired, we try to rejoin. We set the radio to the lowest data rate first.
                        SendResponse::NoAck => info!("No Acknowledgement received."),
                        SendResponse::RxComplete => info!("No data received."),
                        SendResponse::SessionExpired => join_network(&mut device).await,
                    }
                }
                Err(e) => warn!("Unexpected error! {:?}", e),
            }
        }

        //-------------------- Update the values on the flash memory --------------
        {
            let mut counter_values_u64 = [0u64; S0_CHANNEL_COUNT];
            for (i, counter) in S0_COUNTERS.iter().enumerate().take(S0_CHANNEL_COUNT) {
                counter_values_u64[i] = counter.load(Ordering::Relaxed);
            }
            persistent_storage.write(CounterValues {
                counts: counter_values_u64,
            });
        }

        // ----------- Sleep -------
        blinky::PERIOD.signal(Duration::from_millis(2000));

        // Calculate a random delay that is added/subtracted from the sleep duration to prevent transmissions from syncing up and talking over eah other
        let random = embassy_rp::clocks::RoscRng.next_u32();
        let random_frac = random as f32 / u32::MAX as f32; // Range: 0-1
        let random_duration =
            Duration::from_micros((RANDOM_SLEEP_VARIATION.as_micros() as f32 * random_frac) as u64);
        Timer::after(random_duration + MEASUREMENT_TRANSMIT_INTERVAL).await;
    }
}

/// Attempt to join the LoRa network, with an exponential backoff in case of join failure
async fn join_network<R, C, T, G>(device: &mut Device<R, C, T, G>)
where
    R: radio::PhyRxTx + Timings,
    T: radio::Timer,
    C: CryptoFactory + Default,
    G: RngCore,
{
    let mut join_attempt_count = 0;
    loop {
        info!(
            "Joining LoRaWAN network, attempt {:?}",
            join_attempt_count + 1
        );
        // Warning: These values should be unique pre device
        // These are in the order that can be pasted into chirpstack/ttn, the EUIs will be reversed (to LSB)
        // since this is what the rust code expects
        const DEV_EUI: &str = include_str!("../device-config/DEV_EUI");
        const APP_EUI: &str = include_str!("../device-config/APP_EUI");
        const APP_KEY: &str = include_str!("../device-config/APP_KEY");

        // The DEV_EUI and APP_EUI need to be reversed before putting them unto the device, since the default byte order differs
        // The key does not need that, for some reason.
        let mut dev_eui = decode_to_array(DEV_EUI).unwrap();
        dev_eui.reverse();
        let mut app_eui = decode_to_array(APP_EUI).unwrap();
        app_eui.reverse();
        let resp = device
            .join(&JoinMode::OTAA {
                deveui: DevEui::from(dev_eui),
                appeui: AppEui::from(app_eui),
                appkey: AppKey::from(decode_to_array(APP_KEY).unwrap()),
            })
            .await;

        let join_success = match resp {
            Ok(resp) => match resp {
                JoinResponse::JoinSuccess => {
                    info!("LoRa join request successfully accepted.");
                    true
                }
                JoinResponse::NoJoinAccept => {
                    info!("LoRa join request not acknowledged.");
                    false
                }
            },
            Err(e) => {
                warn!("LoRa join request failed with unknown error!: {:?}", e);
                false
            }
        };

        if join_success {
            break;
        }
        //Exponential backoff, up to 2048 seconds
        // Start at 1, then 2, then 4 …
        Timer::after_secs(2_u64.pow(join_attempt_count)).await;
        if join_attempt_count < 11 {
            join_attempt_count += 1
        }
    }
}

#[embassy_executor::task(pool_size = S0_CHANNEL_COUNT)]
async fn counter_task(mut input: Input<'static>, counter_index: usize) -> ! {
    let our_counter = &S0_COUNTERS[counter_index];
    // Wait a bit for any startup noise to be settled
    Timer::after(Duration::from_millis(10)).await;
    loop {
        input.wait_for_high().await;
        our_counter.fetch_add(1, Ordering::Relaxed);
        input.wait_for_low().await;
    }
}

async fn temperature(temp_chan: &mut AdcChannel<'static>, adc: &mut Adc<'static, Async>) -> f32 {
    const SAMPLE_COUNT: usize = 10;

    let mut temperature_results: [u16; SAMPLE_COUNT] = [0; SAMPLE_COUNT];
    for temperature_result in temperature_results.iter_mut() {
        *temperature_result = adc.read(temp_chan).await.unwrap();
        //Sampling delay
        Timer::after_millis(50).await;
    }
    let temperature_result = median(&mut temperature_results);
    let temperature = convert_to_celsius(temperature_result);
    info!("temperature: {:?}", temperature);

    temperature
}

/// Calcualtes the median by sorting the array and taking the middle value
fn median<T>(buf: &mut [T]) -> T
where
    T: Ord + Copy,
{
    // We seriously need to implement a sorting algorithm here.
    for i in 0..buf.len() {
        let mut j = i;
        while j > 0 && buf[j - 1] > buf[j] {
            buf.swap(j - 1, j);
            j -= 1;
        }
    }

    let index_of_middle = (buf.len() - 1) / 2;
    buf[index_of_middle]
}

fn convert_to_celsius(raw_temp: u16) -> f32 {
    // According to chapter 4.9.5. Temperature Sensor in RP2040 datasheet
    let temp = 27.0 - (raw_temp as f32 * 3.3 / 4096.0 - 0.706) / 0.001721;
    let sign = if temp < 0.0 { -1.0 } else { 1.0 };
    let rounded_temp_x10: i16 = ((temp * 10.0) + 0.5 * sign) as i16;
    (rounded_temp_x10 as f32) / 10.0
}
