mod register;

use std::ptr::write;
use byteorder::{BigEndian, ByteOrder};
use embedded_hal::spi::{Error, ErrorKind, SpiBus, SpiDevice};
use crate::register::Register;


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}

struct Flags {
    measurement_initiated: bool,
}

pub struct RM3100<SPI> {
    spi: SPI,
    flags: Flags,

}

// type SPI = dyn SpiDevice<Bus=dyn SpiBus<Error=SensorError>, Error=SensorError>;

impl<SPI> RM3100<SPI>
    where SPI: SpiDevice, SPI::Bus: SpiBus
{
    pub fn new(spi: SPI) -> Self {
        let flags = Flags {
            measurement_initiated: false
        };
        RM3100 { spi, flags }
    }
    fn register_write(&mut self, reg: Register, value: &[u8]) {
        // Make sure SSN has a falling edge and data transmission hasn't already started
        // SSH falling edge, start data transmission
        // Write address, MSB = 0
        let cmd = reg as u8;

        self.spi.write(&[&[cmd], value].concat()).expect("spi Communication Error");
    }

    fn register_read_when_drdy(&mut self, reg: Register, bytes: usize, timeout: u32) -> Result<Vec<u8>, SensorError> {
        // Wait for DRDY on the status byte to go high
        let mut status = None;
        for _ in 0..=timeout {
            // spi.write(&[cmd])?;
            let mut read_buf = [(Register::STATUS as u8) | 0x80, 0x00];
            self.spi.transfer_in_place(&mut read_buf).expect("spi Communication Error");
            if read_buf[1] & 0x80 != 0u8 {
                status = Some(read_buf[1]);
                break;
            }
        }
        if status == None {
            return Err(SensorError::Timeout);
        }

        // Read register address
        let cmd = (reg as u8) | 0x80;

        let mut command_buf = Vec::with_capacity(bytes + 1);
        command_buf.resize(bytes + 1, 0x00);
        command_buf[0] = cmd;

        let mut read_buf = vec![0xFF; bytes + 1];
        self.spi.transfer(read_buf.as_mut_slice(), command_buf.as_slice()).expect("spi Communication Error");
        Ok(read_buf[1..=9].to_vec())
    }

    pub fn set_cycle_count(&mut self, ccx: u16, ccy: u16, ccz: u16) {
        let ccxb = ccx.to_be_bytes();
        let ccyb = ccy.to_be_bytes();
        let cczb = ccz.to_be_bytes();
        let buf = [ccxb[0], ccxb[1], ccyb[0], ccyb[1], cczb[0], cczb[1]];

        self.register_write(Register::CCX, &buf);
    }

    pub fn init_measurement(&mut self, x: bool, y: bool, z: bool) {
        // Generate bitmap for init measurement
        let mut poll_val = 0x00;
        poll_val |= (x as u8) << 4;
        poll_val |= (y as u8) << 5;
        poll_val |= (z as u8) << 6;

        self.register_write(Register::POLL, &[poll_val]);
        self.flags.measurement_initiated = true;
    }

    pub fn read_results(&mut self, x: bool, y: bool, z: bool) -> Result<(i32, i32, i32), SensorError> {
        if !self.flags.measurement_initiated {return Err(SensorError::ReadWithoutInitiateMeasurement)};
        // Read measurement results (nine bytes)
        let result = self.register_read_when_drdy(Register::MX, 9, 128)?;
        let mx_raw = BigEndian::read_i24(&result[0..3]);
        let my_raw = BigEndian::read_i24(&result[3..6]);
        let mz_raw = BigEndian::read_i24(&result[6..9]);

        self.flags.measurement_initiated = false;

        Ok((mx_raw, my_raw, mz_raw))
    }

    // Convert raw value into value in uT (divide by integer range, multiply by sensor's effective range)V
    #[inline(always)]
    pub fn convert_to_uT(raw: i32) -> f64 {
        (raw as f64) / 8388608f64 * 800f64
    }

    pub fn read_convert_all(&mut self) -> (f64, f64, f64){
        let (mx_raw, my_raw, mz_raw) = self.read_results(true, true, true).unwrap();

        let mx = Self::convert_to_uT(mx_raw);
        let my = Self::convert_to_uT(my_raw);
        let mz = Self::convert_to_uT(mz_raw);

        (mx, my, mz)
    }

    pub fn measure_all(&mut self) -> (f64, f64, f64) {
        self.init_measurement(true, true, true);
        self.read_convert_all()
    }
}

#[derive(Copy, Clone, Debug)]
pub enum SensorError {
    Spi,
    Timeout,
    ReadWithoutInitiateMeasurement,
}

impl Error for SensorError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}