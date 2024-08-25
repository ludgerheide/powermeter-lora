#[cfg(feature = "pico_w")]
use cyw43::Control;
#[cfg(feature = "pico_w")]
use cyw43_pio::PioSpi;
use embassy_executor::Spawner;
#[cfg(feature = "pico_w")]
use embassy_rp::bind_interrupts;
#[cfg(feature = "pico_w")]
use embassy_rp::dma::AnyChannel;
#[cfg(feature = "pico_non_w")]
use embassy_rp::gpio::Level::{High, Low};
use embassy_rp::gpio::{Level, Output};
#[cfg(feature = "pico_w")]
use embassy_rp::peripherals::{PIN_23, PIN_24, PIN_29, PIO0};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::with_timeout;
use embassy_time::Duration;
#[cfg(feature = "pico_w")]
use static_cell::StaticCell;

use embassy_rp::peripherals::PIN_25;

#[cfg(feature = "pico_w")]
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});
#[cfg(feature = "pico_non_w")]
use embassy_rp::gpio::Pin;

#[cfg(all(feature = "pico_w", feature = "pico_non_w"))]
compile_error!("Cannot enable code paths for W and non-W hardware simulataenously. Choose one.");

pub static PERIOD: Signal<ThreadModeRawMutex, Duration> = Signal::new();

#[cfg(feature = "pico_w")]
pub struct BlinkPeripherals {
    pub pwr: PIN_23,
    pub cs: PIN_25,
    pub dio: PIN_24,
    pub clk: PIN_29,
    pub dma_ch: AnyChannel,
    pub pio: PIO0,
}

#[cfg(feature = "pico_w")]
pub async fn init(initial_period: Duration, spawner: Spawner, p: BlinkPeripherals) {
    use embassy_rp::pio::Pio;

    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(p.pwr, Level::Low);
    let cs = Output::new(p.cs, Level::High);
    let mut pio = Pio::new(p.pio, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        p.dio,
        p.clk,
        p.dma_ch,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.spawn(cyw43_task(runner)).unwrap();

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::SuperSave)
        .await;

    spawner.spawn(blink_task(control, initial_period)).unwrap();
}

#[cfg(feature = "pico_non_w")]
pub struct BlinkPeripherals {
    pub led: PIN_25,
}

#[cfg(feature = "pico_non_w")]
pub async fn init(initial_period: Duration, spawner: Spawner, p: Peripherals) {
    let led = Output::new(p.PIN_25.degrade(), Level::Low);
    spawner.spawn(blink_task(led, initial_period)).unwrap();
}

#[cfg(feature = "pico_non_w")]
#[embassy_executor::task]
async fn blink_task(mut led: Output<'static>, initial_period: Duration) -> ! {
    let mut current_state = Low;
    let mut period = initial_period;
    loop {
        // Toggle the LED, then either wait for the current frequencies timeout
        // (continuining blinking with the same frequency) or update the frequency
        // right away by taking the signal's value as new frequency
        led.set_level(current_state);
        if current_state == Low {
            current_state = High;
        } else {
            current_state = Low;
        }
        let wait_result = with_timeout(period, PERIOD.wait()).await;
        if let Ok(new_value) = wait_result {
            period = new_value
        }
    }
}

#[cfg(feature = "pico_w")]
#[embassy_executor::task]
async fn blink_task(mut control: Control<'static>, initial_period: Duration) -> ! {
    let mut current_state = false;
    let mut period = initial_period;
    loop {
        // Toggle the LED, then either wait for the current frequencies timeout
        // (continuining blinking with the same frequency) or update the frequency
        // right away by taking the signal's value as new frequency
        control.gpio_set(0, current_state).await;
        current_state = !current_state;
        let wait_result = with_timeout(period, PERIOD.wait()).await;
        if let Ok(new_value) = wait_result {
            period = new_value
        }
    }
}

#[cfg(feature = "pico_w")]
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, AnyChannel>>,
) -> ! {
    runner.run().await
}
