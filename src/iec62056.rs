use core::str::FromStr;

use defmt::{info, trace, warn};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::uart::DataBits::DataBits7;
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUart, Instance, Parity, RxPin, StopBits, TxPin,
};
use embassy_rp::{uart, Peripheral};
use embedded_io_async::Read;
use micromath::F32Ext;
use static_cell::StaticCell;

const UART_BUFFER_SIZE: usize = 255; // In practice, we only get 4 bytes between read calls
const METER_SENTENCE_LENGTH: usize = 64;

#[derive(Copy, Clone, Default)]
pub struct MeterData {
    pub meter_id: u64,
    pub total_in: f32,
    pub total_out: f32,
}

pub struct EnergyMeter<'d, T: Instance> {
    uart: BufferedUart<'d, T>,
}

impl<'d, T: Instance> EnergyMeter<'d, T> {
    /// Sets up the UART
    fn initialize_uart(
        uart: impl Peripheral<P = T> + 'd,
        irq: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>,
        rx: impl Peripheral<P = impl RxPin<T>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T>> + 'd,
    ) -> BufferedUart<'d, T> {
        let mut config = uart::Config::default();
        config.baudrate = 300;
        config.data_bits = DataBits7;
        config.stop_bits = StopBits::STOP1;
        config.parity = Parity::ParityEven;

        static RX_BUF: StaticCell<[u8; UART_BUFFER_SIZE]> = StaticCell::new();
        let rx_buf = &mut RX_BUF.init([0; UART_BUFFER_SIZE])[..];

        static TX_BUF: StaticCell<[u8; UART_BUFFER_SIZE]> = StaticCell::new();
        let tx_buf = &mut TX_BUF.init([0; UART_BUFFER_SIZE])[..];

        BufferedUart::new(uart, irq, tx, rx, tx_buf, rx_buf, config)
    }
    pub fn new(
        uart: impl Peripheral<P = T> + 'd,
        irq: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>,
        rx: impl Peripheral<P = impl RxPin<T>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T>> + 'd,
    ) -> Self {
        let uart = Self::initialize_uart(uart, irq, rx, tx);

        Self { uart }
    }

    /// Uses the UART to synchronize on the start of the sentence and read in a complete sentence
    async fn read_meter_sentence(&mut self, meter_sentence_buf: &mut [u8; METER_SENTENCE_LENGTH]) {
        //Zero out the message buffer
        *meter_sentence_buf = [0; METER_SENTENCE_LENGTH];
        let mut position: usize = 0;
        loop {
            let read_result = self
                .uart
                .read(&mut meter_sentence_buf[position..position + 1])
                .await;
            match read_result {
                Ok(read_count) => {
                    trace!(
                        "RX {:?}",
                        meter_sentence_buf[position..position + read_count]
                    );
                    position += read_count;

                    //Check if the last character read is a linefeed
                    if meter_sentence_buf[position - 1] == b'\n' {
                        return;
                    }
                    // If the buffer is full and we have not gotten a linefeed, clear it
                    if position == meter_sentence_buf.len() {
                        *meter_sentence_buf = [0; METER_SENTENCE_LENGTH];
                        position = 0;
                    }
                }

                Err(_) => warn!("UART Read error encountered!"),
            }
        }
    }

    pub async fn get_data(&mut self) -> MeterData {
        let mut meter_sentence_buf: [u8; METER_SENTENCE_LENGTH] = [0; METER_SENTENCE_LENGTH];
        let mut result = MeterData::default();

        const START_SEQUENCE: &str = "/?!\r\n";
        // Write the start sequence
        self.uart.blocking_write(START_SEQUENCE.as_bytes()).unwrap();

        loop {
            // Read from the serial port until we have a complete sentence in the buffer
            self.read_meter_sentence(&mut meter_sentence_buf).await;

            for in_byte in &mut meter_sentence_buf {
                if *in_byte >= 0x7F {
                    *in_byte = 0x00;
                }
            }

            // Turn it into a string and update the parser
            let sentence = core::str::from_utf8(&meter_sentence_buf).unwrap();
            info!("sentence {:?}", sentence);
            const METER_ID: &str = "C.1";
            const IN: &str = "1.8";
            const OUT: &str = "2.8";

            let first_three_letters = &sentence[0..3];

            match first_three_letters {
                METER_ID => {
                    // The meter  ID is of the format C.1(0000000074892473)
                    // So the fourth character up to the first closing bracket forms the ID
                    match parse_meter_id(sentence) {
                        Some(meter_id) => {
                            result.meter_id = {
                                info!("Meter ID read as {:?}", meter_id);
                                meter_id
                            }
                        }
                        None => warn!("Decoding error!"),
                    }
                }
                IN => {
                    const TOTAL_IN: &str = "1.8.0";
                    const TARIF_1_IN: &str = "1.8.1";
                    const TARIF_2_IN: &str = "1.8.2";

                    let first_five_letters = &sentence[0..5];

                    match first_five_letters {
                        TOTAL_IN => match parse_energy_value(sentence) {
                            Some(energy) => {
                                result.total_in = {
                                    info!("total_in read as {:?}", energy);
                                    energy
                                }
                            }
                            None => warn!("Decoding error!"),
                        },
                        TARIF_1_IN => info!("Contains Tarif 1"),
                        TARIF_2_IN => info!("Contains Tarif 2"),
                        &_ => {}
                    }
                    return result;
                }
                OUT => {
                    match parse_energy_value(sentence) {
                        Some(energy) => {
                            result.total_out = {
                                info!("total_out read as {:?}", energy);
                                energy
                            }
                        }
                        None => warn!("Decoding error!"),
                    }

                    // At this stage, we don't care about the rest of the message
                    return result;
                }
                &_ => {}
            }
        }
    }
}

fn parse_meter_id(sentence: &str) -> Option<u64> {
    // Find the start of the numeric value within the parentheses
    if let Some(start) = sentence.find('(') {
        if let Some(end) = sentence[start..].find(')') {
            let numeric_part = &sentence[start + 1..start + end];
            // Try to parse the numeric part as a u64
            match u64::from_str(numeric_part) {
                Ok(meter_id) => return Some(meter_id),
                Err(_) => return None,
            }
        }
    }
    None
}

fn parse_energy_value(sentence: &str) -> Option<f32> {
    // Find the start and end of the numerical value within the parentheses
    if let Some(start) = sentence.find('(') {
        if let Some(end) = sentence[start..].find('*') {
            let numeric_part = &sentence[start + 1..start + end];
            // Split the numeric part at the decimal point
            if let Some(dot_index) = numeric_part.find('.') {
                let (int_part, frac_part) = numeric_part.split_at(dot_index);
                // Parse integral part
                if let Ok(int_val) = u32::from_str(int_part) {
                    // Remove the decimal point for fractional part and parse
                    let frac_part = &frac_part[1..]; // Skip the dot
                    if let Ok(frac_val) = u32::from_str(frac_part) {
                        let frac_len = frac_part.len() as u32;
                        // Calculate the float value
                        return Some(
                            int_val as f32 + frac_val as f32 / F32Ext::powi(10f32, frac_len as i32),
                        );
                    }
                }
            }
        }
    }
    None
}
