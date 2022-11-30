use std::borrow::Borrow;
use std::cell::RefCell;
use std::convert::TryInto;
use std::sync::{Arc, Mutex};
use std::env;
use std::rc::Rc;
use shared_bus::{BusManager, BusManagerSimple, I2cProxy, NullMutex};

use pni_rm3100::RM3100;
use mcp23017::{MCP23017, PinMode, Port};
use rppal::i2c::I2c;
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
use embedded_hal::digital::{ErrorType, OutputPin};
use embedded_hal_bus::spi::ExclusiveDevice;

use anyhow::{Error, Result};

#[derive(Copy, Clone, Debug)]
pub enum CSError {
    OutputPin
}

pub struct SoftwareOutputPin {
    high: Box<dyn Fn() -> Result<(), CSError>>,
    low: Box<dyn Fn() -> Result<(), CSError>>,
}


impl ErrorType for SoftwareOutputPin { type Error = CSError; }

impl OutputPin for SoftwareOutputPin {
    fn set_low(&mut self) -> std::result::Result<(), Self::Error> {
        (self.low)()
    }

    fn set_high(&mut self) -> std::result::Result<(), Self::Error> {
        (self.high)()
    }
}


pub struct CSDelegator
{
    mcp: MCP23017<I2c>,
}

impl CSDelegator {
    pub fn new(mcp: MCP23017<I2c>, initial_value: u16) -> Self {
        let mut new = CSDelegator { mcp };
        new.mcp.all_pin_mode(PinMode::OUTPUT).unwrap();
        new.mcp.write_gpioab(initial_value).unwrap();
        new
    }

    #[inline(always)]
    pub fn cs_high(&mut self, index: u8) {
        self.set_cs(index, true).unwrap();
    }
    #[inline(always)]
    pub fn cs_low(&mut self, index: u8) {
        self.set_cs(index, false).unwrap();
    }

    /// Toggles selected CS pin on magnetometer array
    /// # Arguments
    /// * `index` - From 0 to 15, represents the index of the magnetometer
    fn set_cs(&mut self, index: u8, state: bool) -> Result<(), CSError> {
        let port = if index & 0x08 != 0 { Port::GPIOB } else { Port::GPIOA };
        let bitmask =
            if !state {
                match port {
                    Port::GPIOA => !(0x80 >> (index & 0x07)),
                    Port::GPIOB => !(0x01 << (index & 0x07))
                }
            } else {
                0xFF
            };


        match self.mcp.write_gpio(port, bitmask) {
            Ok(T) => Ok(T),
            Err(E) => Err(CSError::OutputPin)
        }
    }

    pub fn get_output_pin(manager: &Rc<Mutex<Self>>, index: u8) -> SoftwareOutputPin {
        let m1 = manager.clone();
        let m2 = manager.clone();
        let high = move || {
            m1.try_lock().unwrap().set_cs(index.clone(), true)
        };
        let low = move || {
            m2.try_lock().unwrap().set_cs(index.clone(), false)
        };
        SoftwareOutputPin { high: Box::new(high), low: Box::new(low) }
    }
}
