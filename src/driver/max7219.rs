//! Core MAX7219 driver implementation

use embedded_hal::spi::SpiDevice;

use crate::{
    MAX_DISPLAYS, NUM_DIGITS, Result,
    error::Error,
    registers::{DecodeMode, Register},
};

/// Driver for the MAX7219 LED display controller.
/// Communicates over SPI using the embedded-hal `SpiDevice` trait.
pub struct Max7219<SPI> {
    spi: SPI,
    buffer: [u8; MAX_DISPLAYS * 2],
    device_count: usize,
}

impl<SPI> Max7219<SPI>
where
    SPI: SpiDevice,
{
    /// Creates a new MAX7219 driver instance with the given SPI interface.
    ///
    /// The SPI interface must use Mode 0, which means the clock is low when idle
    /// and data is read on the rising edge of the clock signal.
    ///
    /// Defaults to a single device (can be daisy-chained using `with_device_count`).
    ///
    /// The SPI frequency must be 10 MHz or less, as required by the MAX7219 datasheet.
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            device_count: 1, // Default to 1, use with_device_count to increase count
            buffer: [0; MAX_DISPLAYS * 2],
        }
    }

    /// Returns the number of MAX7219 devices managed by this driver.
    ///
    /// This corresponds to the number of daisy-chained MAX7219 units
    /// initialized during driver setup.
    pub fn device_count(&self) -> usize {
        self.device_count
    }

    /// Sets the number of daisy-chained devices to control.
    ///
    /// # Errors
    ///
    /// Returns `Error::InvalidDisplayCount` if `count > MAX_DISPLAYS`.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let driver = Max7219::new(spi).with_device_count(4)?;
    /// ```
    pub fn with_device_count(mut self, count: usize) -> Result<Self> {
        if count > MAX_DISPLAYS {
            return Err(Error::InvalidDeviceCount);
        }
        self.device_count = count;
        Ok(self)
    }

    /// Initializes all configured displays.
    pub fn init(&mut self) -> Result<()> {
        self.power_on()?;

        self.test_all(false)?;
        self.set_scan_limit_all(NUM_DIGITS)?;
        self.set_decode_mode_all(DecodeMode::NoDecode)?;

        self.clear_all()?;
        // self.power_off()?;
        // self.power_on()?;

        Ok(())
    }

    /// Writes a value to a specific register of a device in the daisy chain.
    ///
    /// Each MAX7219 device expects a 16-bit packet: 1 byte for the register address
    /// and 1 byte for the data. To update one device in a daisy-chained series,
    /// we prepare a full SPI buffer of `display_count * 2` bytes (2 bytes per display).
    ///
    /// This method writes the target register and data into the correct offset of
    /// the shared buffer corresponding to the selected device (`device_index`),
    /// and clears the rest of the buffer. Then the entire buffer is sent via SPI.
    ///
    /// The device at `device_index` will receive its register update, while other
    /// devices in the chain will receive no-ops (zeros).
    ///
    /// # Arguments
    ///
    /// * `device_index` - Index of the device in the chain (0 = furthest from MCU, N-1 = closest to MCU).
    /// * `register` - The register to write to (e.g., `Register::Shutdown`, `Register::Digit0`, etc.).
    /// * `data` - The value to write to the register.
    ///
    /// # Errors
    ///
    /// Returns `Error::InvalidDisplayIndex` if the index is out of range, or an SPI error
    /// if the transfer fails.
    pub(crate) fn write_device_register(
        &mut self,
        device_index: usize,
        register: Register,
        data: u8,
    ) -> Result<()> {
        if device_index >= self.device_count {
            return Err(Error::InvalidDeviceIndex);
        }

        self.buffer = [0; MAX_DISPLAYS * 2];

        let offset = device_index * 2; // 2 bytes(16 bits packet) per display
        self.buffer[offset] = register as u8;
        self.buffer[offset + 1] = data;

        self.spi.write(&self.buffer[0..self.device_count * 2])?;

        Ok(())
    }

    /// Write each (register, data) tuple to its corresponding MAX7219 device in the daisy chain.
    ///
    /// The number of tuples in `ops` must exactly match `self.device_count`.
    /// Convention: ops[0] = furthest device from MCU, ops[device_count-1] = nearest device
    /// Because The first one we send in the SPI gets pushed till the last device.
    ///
    /// # Panics (only in debug builds)
    /// - If `ops.len() != self.device_count`.
    ///
    /// # Errors
    /// - Returns an SPI error if the write operation fails.
    pub(crate) fn write_all_registers(&mut self, ops: &[(Register, u8)]) -> Result<()> {
        // clear the buffer: 2 bytes per device
        self.buffer = [0; MAX_DISPLAYS * 2];

        for (i, &(reg, data)) in ops.iter().enumerate() {
            let offset = i * 2;
            self.buffer[offset] = reg as u8;
            self.buffer[offset + 1] = data;
        }

        // send exactly device_count packets
        let len = self.device_count * 2;
        self.spi.write(&self.buffer[..len])?;

        Ok(())
    }

    // fn write_raw_register(&mut self, register: u8, data: u8) -> Result<(), SPI::Error> {
    //     self.spi.write(&[register, data])
    // }

    /// Powers on all displays by writing `0x01` to the Shutdown register.
    pub fn power_on(&mut self) -> Result<()> {
        let ops = [(Register::Shutdown, 0x01); MAX_DISPLAYS];

        self.write_all_registers(&ops[..self.device_count])
    }

    /// Powers off all displays by writing `0x00` to the Shutdown register.
    pub fn power_off(&mut self) -> Result<()> {
        let ops = [(Register::Shutdown, 0x00); MAX_DISPLAYS];

        self.write_all_registers(&ops[..self.device_count])
    }

    /// Powers on a single device by writing `0x01` to the Shutdown register.
    ///
    /// # Arguments
    ///
    /// * `device_index` - The index of the display to power on.
    pub fn power_on_device(&mut self, device_index: usize) -> Result<()> {
        self.write_device_register(device_index, Register::Shutdown, 0x01)
    }

    /// Powers off a single device by writing `0x00` to the Shutdown register.
    ///
    /// # Arguments
    ///
    /// * `device_index` - The index of the device to power off.
    pub fn power_off_device(&mut self, device_index: usize) -> Result<()> {
        self.write_device_register(device_index, Register::Shutdown, 0x00)
    }

    /// Enables or disables display test mode on a specific device.
    ///
    /// When enabled, all LEDs on that device are lit regardless of current device data.
    pub fn test_device(&mut self, device_index: usize, enable: bool) -> Result<()> {
        let data = if enable { 0x01 } else { 0x00 };
        self.write_device_register(device_index, Register::DisplayTest, data)
    }

    /// Enable or disable display test mode on all devices in one SPI transaction.
    pub fn test_all(&mut self, enable: bool) -> Result<()> {
        let data = if enable { 0x01 } else { 0x00 };
        let ops: [(Register, u8); MAX_DISPLAYS] = [(Register::DisplayTest, data); MAX_DISPLAYS];
        self.write_all_registers(&ops[..self.device_count])
    }

    /// Sets how many digits the MAX7219 should actively scan and display.
    ///
    /// This tells the chip how many digit outputs (DIG0 to DIG7) should be used.
    /// The input value must be between 1 and 8:
    /// - 1 means only digit 0 is used
    /// - 8 means all digits (0 to 7) are used
    ///
    /// Internally, the value written to the chip is `limit - 1`, because the chip expects values from 0 to 7.
    ///
    /// This applies to a specific device in the daisy chain, selected by `device_index`.
    ///
    /// # Errors
    /// Returns `Error::InvalidScanLimit` if the value is not in the range 1 to 8.
    pub fn set_device_scan_limit(&mut self, device_index: usize, limit: u8) -> Result<()> {
        if !(1..=8).contains(&limit) {
            return Err(Error::InvalidScanLimit);
        }

        self.write_device_register(device_index, Register::ScanLimit, limit - 1)
    }

    /// Set scan‐limit on all devices in one go.
    ///
    /// `limit` must be in 1..=8. Internally sends `limit - 1` to each chip.
    pub fn set_scan_limit_all(&mut self, limit: u8) -> Result<()> {
        if !(1..=8).contains(&limit) {
            return Err(Error::InvalidScanLimit);
        }
        let val = limit - 1;
        let ops: [(Register, u8); MAX_DISPLAYS] = [(Register::ScanLimit, val); MAX_DISPLAYS];
        self.write_all_registers(&ops[..self.device_count])
    }

    /// Code B decoding allows the MAX7219 to automatically convert values like `0-9`, `E`, `H`, `L`, etc.
    /// into the corresponding 7-segment patterns, instead of requiring manual segment control.
    ///
    /// The `mode` parameter specifies which digits use automatic decoding.
    /// Use [`DecodeMode`] variants
    /// such as [`NoDecode`], [`Digit0`], [`Digits0To3`], or [`AllDigits`] based on which digits
    /// should be decoded automatically.
    ///
    /// The `device_index` selects the target device. For a single device setup, use `0`.
    pub fn set_device_decode_mode(&mut self, device_index: usize, mode: DecodeMode) -> Result<()> {
        self.write_device_register(device_index, Register::DecodeMode, mode as u8)
    }

    /// Set decode‐mode on all devices in one go.
    pub fn set_decode_mode_all(&mut self, mode: DecodeMode) -> Result<()> {
        let byte = mode as u8;
        let ops: [(Register, u8); MAX_DISPLAYS] = [(Register::DecodeMode, byte); MAX_DISPLAYS];
        self.write_all_registers(&ops[..self.device_count])
    }

    /// Clears all digits by writing 0 to each digit register (DIG0 to DIG7).
    ///
    /// This turns off all segments on the display by sending 0x00 to each of the
    /// digit registers (Register::Digit0 to Register::Digit7).
    ///
    /// This applies to a specific device in the daisy chain, selected by `device_index`.
    pub fn clear_display(&mut self, device_index: usize) -> Result<()> {
        for digit_register in Register::digits() {
            self.write_device_register(device_index, digit_register, 0x00)?;
        }
        Ok(())
    }

    /// Clears all digits on all connected MAX7219 displays.
    pub fn clear_all(&mut self) -> Result<()> {
        for digit_register in Register::digits() {
            let ops = [(digit_register, 0x00); MAX_DISPLAYS];
            self.write_all_registers(&ops[..self.device_count])?;
        }

        Ok(())
    }

    /// Writes a raw value to the specified digit register (DIG0 to DIG7).
    ///
    /// This function gives you low-level control over the display by sending a
    /// raw 8-bit pattern to the specified digit. Each bit in the `value` corresponds
    /// to an individual segment (on 7-segment displays) or LED (on an LED matrix).
    ///
    /// **A typical 7-segment** display has the following layout:
    ///
    /// ```txt
    ///     A
    ///    ---
    /// F |   | B
    ///   |   |
    ///    ---
    /// E |   | C
    ///   |   |
    ///    ---   . DP
    ///     D
    /// ```
    ///
    /// | Byte        | 7  | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
    /// |-------------|----|---|---|---|---|---|---|---|
    /// | **Segment** | DP | A | B | C | D | E | F | G |
    ///
    /// For example, to display the number `1`, use the byte `0b00110000`,
    /// which lights up segments B and C.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// display.write_raw_digit(0, Digit::D0, 0b00110000)?; // Shows '1'
    /// ```
    ///
    /// **On an LED matrix (8x8)**, each digit register maps to a row, and each
    /// bit in the value maps to a column (from left to right).
    ///
    /// > **Note:** Wiring and orientation can vary between modules. Some modules map rows top-to-bottom,
    /// > others bottom-to-top; some wire columns left-to-right, others right-to-left.
    /// > If the display appears mirrored or rotated, adjust your digit and bit mapping accordingly.
    ///
    /// Here is an example layout for the FC-16 module, where DIG0 corresponds to the top row (row 0),
    /// and bit 0 maps to the rightmost column (column 0). So a value like `0b10101010` written to DIG0
    /// would light up every alternate LED across the top row from left to right.
    ///
    /// ```txt
    /// DIG0 -> Row 0: value = 0b10101010
    ///
    /// Matrix:
    ///           Columns
    ///            7 6 5 4 3 2 1 0
    ///          +----------------
    ///      0   | 1 0 1 0 1 0 1 0
    ///      1   | ...
    ///      2   | ...
    /// Rows 3   | ...
    ///      4   | ...
    ///      5   | ...
    ///      6   | ...
    ///      7   | ...
    /// ```
    ///
    /// This applies to a specific device in the daisy chain, selected by `device_index`.
    ///
    /// # Arguments
    ///
    /// - `device_index`: Index of the display in the daisy chain (0 = Furthest from the Microcontroller)
    /// - `digit`: Which digit register to write to (`Digit::D0` to `Digit::D7`)
    /// - `value`: The raw 8-bit data to send to the digit register
    pub fn write_raw_digit(&mut self, device_index: usize, digit: u8, value: u8) -> Result<()> {
        let digit_register = Register::try_digit(digit)?;
        self.write_device_register(device_index, digit_register, value)
    }

    /// Sets the brightness intensity (0 to 15) for a specific device.
    ///
    /// # Arguments
    ///
    /// - `device_index`: Index of the display in the daisy chain (0 = Furthest from the Microcontroller)
    /// - `intensity`: Brightness level from `0` to `15` (`0x00` to `0x0F`)
    pub fn set_intensity(&mut self, device_index: usize, intensity: u8) -> Result<()> {
        if intensity > 0x0F {
            return Err(Error::InvalidIntensity);
        }
        self.write_device_register(device_index, Register::Intensity, intensity)
    }

    /// Set intensity for all displays
    pub fn set_intensity_all(&mut self, intensity: u8) -> Result<()> {
        if intensity > 0x0F {
            return Err(Error::InvalidIntensity);
        }
        let ops = [(Register::Intensity, intensity); MAX_DISPLAYS];
        self.write_all_registers(&ops[..self.device_count])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_DISPLAYS, NUM_DIGITS, registers::DecodeMode, registers::Register};
    use embedded_hal_mock::eh1::{spi::Mock as SpiMock, spi::Transaction};

    #[test]
    fn test_new() {
        let mut spi = SpiMock::new(&[]);
        let driver = Max7219::new(&mut spi);
        // Default device count => 1
        assert_eq!(driver.device_count(), 1);

        spi.done();
    }

    #[test]
    fn test_with_device_count_valid() {
        let mut spi = SpiMock::new(&[]);
        let driver = Max7219::new(&mut spi);
        let driver = driver
            .with_device_count(4)
            .expect("Should accept valid count");
        assert_eq!(driver.device_count(), 4);
        spi.done();
    }

    #[test]
    fn test_with_device_count_invalid() {
        let mut spi = SpiMock::new(&[]);
        let driver = Max7219::new(&mut spi);
        let result = driver.with_device_count(MAX_DISPLAYS + 1);
        assert!(matches!(result, Err(Error::InvalidDeviceCount)));

        spi.done();
    }

    #[test]
    fn test_power_on() {
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Shutdown.addr(), 0x01]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver.power_on().expect("Power on should succeed");
        spi.done();
    }

    #[test]
    fn test_power_off() {
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Shutdown.addr(), 0x00]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver.power_off().expect("Power off should succeed");
        spi.done();
    }

    #[test]
    fn test_power_on_device() {
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Shutdown.addr(), 0x01]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver
            .power_on_device(0)
            .expect("Power on display should succeed");
        spi.done();
    }

    // Test with multiple devices - power_on
    #[test]
    fn test_power_on_multiple_devices() {
        let device_count = 3;

        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![
                Register::Shutdown.addr(),
                0x01,
                Register::Shutdown.addr(),
                0x01,
                Register::Shutdown.addr(),
                0x01,
            ]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi)
            .with_device_count(device_count)
            .expect("Should accept valid count");

        driver.power_on().expect("Power on should succeed");
        spi.done();
    }

    #[test]
    fn test_power_off_device() {
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![
                // For 4 devices
                Register::NoOp.addr(),
                0x00,
                Register::NoOp.addr(),
                0x00,
                Register::Shutdown.addr(),
                0x00,
                Register::NoOp.addr(),
                0x00,
            ]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi)
            .with_device_count(4)
            .expect("a valid device count");

        driver
            .power_off_device(2)
            .expect("Power off display should succeed");
        spi.done();
    }

    #[test]
    fn test_power_device_invalid_index() {
        let mut spi = SpiMock::new(&[]);
        let mut driver = Max7219::new(&mut spi).with_device_count(1).unwrap();

        let result = driver.power_on_device(1);
        assert_eq!(result, Err(Error::InvalidDeviceIndex));

        let result = driver.power_off_device(1);
        assert_eq!(result, Err(Error::InvalidDeviceIndex));
        spi.done();
    }

    #[test]
    fn test_test_all_enable() {
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![
                Register::DisplayTest.addr(),
                0x01,
                Register::DisplayTest.addr(),
                0x01,
                Register::DisplayTest.addr(),
                0x01,
                Register::DisplayTest.addr(),
                0x01,
            ]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi)
            .with_device_count(4)
            .expect("valid device count");

        driver
            .test_all(true)
            .expect("Test all enable should succeed");
        spi.done();
    }

    #[test]
    fn test_test_all_disable() {
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::DisplayTest.addr(), 0x00]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver
            .test_all(false)
            .expect("Test all disable should succeed");
        spi.done();
    }

    #[test]
    fn test_set_scan_limit_all_valid() {
        let limit = 4;
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::ScanLimit.addr(), limit - 1]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver
            .set_scan_limit_all(limit)
            .expect("Set scan limit should succeed");
        spi.done();
    }

    #[test]
    fn test_set_scan_limit_all_invalid_low() {
        let mut spi = SpiMock::new(&[]);
        let mut driver = Max7219::new(&mut spi);

        let result = driver.set_scan_limit_all(0);
        assert_eq!(result, Err(Error::InvalidScanLimit));
        spi.done();
    }

    #[test]
    fn test_set_scan_limit_all_invalid_high() {
        let mut spi = SpiMock::new(&[]); // No transactions expected for invalid input
        let mut driver = Max7219::new(&mut spi);

        let result = driver.set_scan_limit_all(9);
        assert_eq!(result, Err(Error::InvalidScanLimit));
        spi.done();
    }

    #[test]
    fn test_set_decode_mode_all() {
        let mode = DecodeMode::AllDigits;
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![
                Register::DecodeMode.addr(),
                mode.value(),
                Register::DecodeMode.addr(),
                mode.value(),
                Register::DecodeMode.addr(),
                mode.value(),
                Register::DecodeMode.addr(),
                mode.value(),
            ]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi)
            .with_device_count(4)
            .expect("valid device count");

        driver
            .set_decode_mode_all(mode)
            .expect("Set decode mode should succeed");
        spi.done();
    }

    #[test]
    fn test_clear_display() {
        let mut expected_transactions = Vec::new();
        for digit_register in Register::digits() {
            expected_transactions.push(Transaction::transaction_start());
            expected_transactions.push(Transaction::write_vec(vec![digit_register.addr(), 0x00]));
            expected_transactions.push(Transaction::transaction_end());
        }

        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver
            .clear_display(0)
            .expect("Clear display should succeed");
        spi.done();
    }

    #[test]
    fn test_clear_display_invalid_index() {
        let mut spi = SpiMock::new(&[]); // No transactions expected for invalid index
        let mut driver = Max7219::new(&mut spi)
            .with_device_count(1)
            .expect("valid device count");

        let result = driver.clear_display(1);
        assert_eq!(result, Err(Error::InvalidDeviceIndex));
        spi.done();
    }

    #[test]
    fn test_write_raw_digit() {
        let device_index = 0;
        let digit = 3;
        let data = 0b10101010;
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Digit3.addr(), data]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver
            .write_raw_digit(device_index, digit, data)
            .expect("Write raw digit should succeed");
        spi.done();
    }

    #[test]
    fn test_write_raw_digit_invalid_digit() {
        let mut spi = SpiMock::new(&[]); // No transactions expected for invalid digit
        let mut driver = Max7219::new(&mut spi);

        let result = driver.write_raw_digit(0, 8, 0x00); // Digit 8 is invalid

        assert_eq!(result, Err(Error::InvalidDigit));

        spi.done();
    }

    #[test]
    fn test_set_intensity_valid() {
        let device_index = 0;
        let intensity = 0x0A;
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Intensity.addr(), intensity]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver
            .set_intensity(device_index, intensity)
            .expect("Set intensity should succeed");
        spi.done();
    }

    #[test]
    fn test_set_intensity_invalid() {
        let mut spi = SpiMock::new(&[]); // No transactions expected for invalid input
        let mut driver = Max7219::new(&mut spi);

        let result = driver.set_intensity(0, 0x10); // Invalid intensity > 0x0F
        assert_eq!(result, Err(Error::InvalidIntensity));
        spi.done();
    }

    #[test]
    fn test_init() {
        // Mock the sequence of calls made by init() for 1 device
        // 1. power_on() -> Shutdown 0x01
        // 2. test_all(false) -> DisplayTest 0x00
        // 3. set_scan_limit_all(NUM_DIGITS) -> ScanLimit (NUM_DIGITS-1)
        // 4. set_decode_mode_all(NoDecode) -> DecodeMode 0x00
        // 5. clear_all() -> 8 separate calls to write_all_registers for each digit reg

        // Use vec![] macro to create the vector with all expected transactions
        let expected_transactions = vec![
            // 1. power_on (write_all_registers)
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Shutdown.addr(), 0x01]),
            Transaction::transaction_end(),
            // 2. test_all(false) (write_all_registers)
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::DisplayTest.addr(), 0x00]),
            Transaction::transaction_end(),
            // 3. set_scan_limit_all (write_all_registers)
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::ScanLimit.addr(), NUM_DIGITS - 1]),
            Transaction::transaction_end(),
            // 4. set_decode_mode_all (write_all_registers)
            Transaction::transaction_start(),
            Transaction::write_vec(vec![
                Register::DecodeMode.addr(),
                DecodeMode::NoDecode.value(),
            ]),
            Transaction::transaction_end(),
            // 5. clear_all() - 8 separate write_all_registers calls, one for each digit reg
            // Unroll the loop for clarity and to include all transactions in the vec![] macro
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Digit0.addr(), 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Digit1.addr(), 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Digit2.addr(), 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Digit3.addr(), 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Digit4.addr(), 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Digit5.addr(), 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Digit6.addr(), 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::Digit7.addr(), 0x00]),
            Transaction::transaction_end(),
        ];

        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver.init().expect("Init should succeed");
        spi.done();
    }

    #[test]
    fn test_write_device_register_valid_index() {
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![
                Register::Shutdown.addr(),
                0x01,
                0x00, // no-op for second device in chain
                0x00,
            ]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi)
            .with_device_count(2)
            .expect("Should accept valid count");

        driver
            .write_device_register(0, Register::Shutdown, 0x01)
            .expect("should write register");

        spi.done();
    }

    #[test]
    fn test_write_device_register_invalid_index() {
        let mut spi = SpiMock::new(&[]); // No SPI transactions expected
        let mut driver = Max7219::new(&mut spi)
            .with_device_count(2)
            .expect("Should accept valid count");

        let result = driver.write_device_register(2, Register::Shutdown, 0x01); // Index 2 is invalid for device_count=2
        assert_eq!(result, Err(Error::InvalidDeviceIndex));

        spi.done();
    }

    #[test]
    fn test_write_all_registers_valid() {
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![
                Register::Intensity.addr(),
                0x01,
                Register::Intensity.addr(),
                0x01,
            ]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi)
            .with_device_count(2)
            .expect("Should accept valid count");

        driver
            .write_all_registers(&[(Register::Intensity, 0x01), (Register::Intensity, 0x01)])
            .expect("should  write all registers");

        spi.done();
    }

    #[test]
    fn test_test_device_enable_disable() {
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::DisplayTest.addr(), 0x01]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::DisplayTest.addr(), 0x00]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver
            .test_device(0, true)
            .expect("Enable test mode failed");
        driver
            .test_device(0, false)
            .expect("Disable test mode failed");
        spi.done();
    }

    #[test]
    fn test_set_device_scan_limit_valid() {
        let scan_limit = 4;

        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::ScanLimit.addr(), scan_limit - 1]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver
            .set_device_scan_limit(0, scan_limit)
            .expect("Scan limit set failed");
        spi.done();
    }

    #[test]
    fn test_set_device_scan_limit_invalid() {
        let mut spi = SpiMock::new(&[]);
        let mut driver = Max7219::new(&mut spi);

        let result = driver.set_device_scan_limit(0, 0); // invalid: below range
        assert_eq!(result, Err(Error::InvalidScanLimit));

        let result = driver.set_device_scan_limit(0, 9); // invalid: above range
        assert_eq!(result, Err(Error::InvalidScanLimit));
        spi.done();
    }

    #[test]
    fn test_set_device_decode_mode() {
        let mode = DecodeMode::Digits0To3;
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![Register::DecodeMode.addr(), mode.value()]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi);

        driver
            .set_device_decode_mode(0, mode)
            .expect("Set decode mode failed");
        spi.done();
    }

    #[test]
    fn test_set_intensity_all() {
        let intensity = 0x05;
        let expected_transactions = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![
                Register::Intensity.addr(),
                intensity,
                Register::Intensity.addr(),
                intensity,
            ]),
            Transaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expected_transactions);
        let mut driver = Max7219::new(&mut spi)
            .with_device_count(2)
            .expect("valid count");

        driver
            .set_intensity_all(intensity)
            .expect("Set intensity all failed");
        spi.done();
    }

    #[test]
    fn test_set_intensity_all_invalid() {
        let mut spi = SpiMock::new(&[]); // No transactions expected for invalid input
        let mut driver = Max7219::new(&mut spi);

        let result = driver.set_intensity_all(0x10); // Invalid intensity > 0x0F
        assert_eq!(result, Err(Error::InvalidIntensity));
        spi.done();
    }
}
