mod chip_select;

use std::borrow::Borrow;
use std::cell::RefCell;
use std::convert::TryInto;
use std::sync::{Arc, Mutex};
use std::{env, thread, time};
use std::ptr::null_mut;
use std::rc::Rc;
use std::sync::mpsc;
use threadpool::ThreadPool;

use rclrs::Context;
use geometry_msgs::msg::Vector3;
use sensor_msgs::msg::MagneticField;
use std_msgs::msg::Header;
use mag_array_interfaces::msg::MagArrayFrame;

use shared_bus::{BusManager, BusManagerSimple, I2cProxy, NullMutex};

use pni_rm3100::RM3100;
use mcp23017::{MCP23017, PinMode};
use rppal::i2c::I2c;
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
use embedded_hal::digital::{ErrorType, OutputPin};
use embedded_hal_bus::spi::ExclusiveDevice;

use anyhow::{Error, Result};
use embedded_hal::spi::SpiBus;

use crate::chip_select::CSDelegator;
use crate::ServiceThreadCommands::Measure;


enum ServiceThreadCommands {
    Measure,
    Kill,
}


fn main() -> Result<(), Error> {
    const NUM_SENSORS: (usize, usize) = (16, 11);
    const BANK_OFFSETS: (usize, usize) = (0, 16);

    const TOTAL_SENSORS: usize = NUM_SENSORS.0 + NUM_SENSORS.1;
    dbg!("hello");

    let (results_tx, results_rx) = mpsc::channel();
    let results_txc0 = results_tx.clone();
    let results_txc1 = results_tx.clone();

    let (service0_tx, service0_rx) = mpsc::channel();
    let (service1_tx, service1_rx) = mpsc::channel();

    let service_thread0 = thread::spawn(move || {
        // Bus initialization
        let mut mcp = MCP23017::new(I2c::with_bus(5).unwrap(), 0x20).unwrap();
        let mut chip_selector = Rc::new(Mutex::new(CSDelegator::new(mcp, 0xFFFF)));
        let spi_shared = BusManagerSimple::new(Spi::new(Bus::Spi1, SlaveSelect::Ss0, 200_000, Mode::Mode0).unwrap());

        // Sensor hardware initialization
        let mut sensor_vec = Vec::with_capacity(NUM_SENSORS.0);
        for x in 0..NUM_SENSORS.0 {
            let mut output = CSDelegator::get_output_pin(&chip_selector, x as u8);
            let mut rm3100 = RM3100::new(ExclusiveDevice::new(spi_shared.acquire_spi(), output));
            rm3100.set_cycle_count(100, 100, 100);
            sensor_vec.push(rm3100);
        }

        // Thread service provider
        loop {
            // Wait for a request - either measure or kill
            match service0_rx.recv().unwrap() {
                Measure => {
                    for x in 0..NUM_SENSORS.0 {
                        // println!("Measuring Bank 0 Sensor {}", x);
                        sensor_vec[x].init_measurement(true, true, true);
                    }
                    for x in 0..NUM_SENSORS.0 {
                        let (mx, my, mz) = sensor_vec[x].read_convert_all();
                        // println!("Bank 0: Sensor {}: {}, {}, {}", x, mx, my, mz);
                        results_txc0.send((x + BANK_OFFSETS.0, mx, my, mz)).unwrap();
                    }
                }
                Kill => break
            }
        }
    });


    let service_thread1 = thread::spawn(move || {
        // Bus initialization
        let mut mcp = MCP23017::new(I2c::with_bus(6).unwrap(), 0x20).unwrap();
        let mut chip_selector = Rc::new(Mutex::new(CSDelegator::new(mcp, 0xFFFF)));
        let spi_shared = BusManagerSimple::new(Spi::new(Bus::Spi0, SlaveSelect::Ss0, 200_000, Mode::Mode0).unwrap());

        // Sensor hardware initialization
        let mut sensor_vec = Vec::with_capacity(NUM_SENSORS.1);
        for x in 0..NUM_SENSORS.1 {
            let mut output = CSDelegator::get_output_pin(&chip_selector, x as u8);
            let mut rm3100 = RM3100::new(ExclusiveDevice::new(spi_shared.acquire_spi(), output));
            rm3100.set_cycle_count(100, 100, 100);
            sensor_vec.push(rm3100);
        }

        // Thread service provider
        loop {
            // Wait for a request - either measure or kill
            match service1_rx.recv().unwrap() {
                Measure => {
                    for x in 0..NUM_SENSORS.1 {
                        // println!("Measuring Bank 1 Sensor {}", x);
                        sensor_vec[x].init_measurement(true, true, true);
                    }
                    for x in 0..NUM_SENSORS.1 {
                        let (mx, my, mz) = sensor_vec[x].read_convert_all();
                        // println!("Bank 1: Sensor {}: {}, {}, {}", x, mx, my, mz);
                        results_txc1.send((x + BANK_OFFSETS.1, mx, my, mz)).unwrap();
                    }
                }
                Kill => break
            }
        }
    });

    let context = Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "mag_array")?;
    let publisher = node.create_publisher::<MagArrayFrame>("mag_array", rclrs::QOS_PROFILE_DEFAULT).unwrap();


    loop {
        service0_tx.send(Measure).unwrap();
        service1_tx.send(Measure).unwrap();
        let mut measurements = Vec::with_capacity(TOTAL_SENSORS);

        for s in 0..TOTAL_SENSORS {
            measurements.push(MagneticField::default());
        }


        let mut frame_header = Header::default();
        let timestamp = time::SystemTime::now().duration_since(time::UNIX_EPOCH).unwrap();
        frame_header.stamp.sec = timestamp.as_secs() as i32;
        frame_header.stamp.nanosec = timestamp.subsec_nanos() as u32;

        let mut message = MagArrayFrame::default();

        message.header = frame_header;

        for s in 0..TOTAL_SENSORS {
            let (ind, x, y, z) = results_rx.recv().unwrap();
            // Do some stuff to put this into proper ROS message to be passed to node
            // Example implementation:


            let timestamp = time::SystemTime::now().duration_since(time::UNIX_EPOCH).unwrap();
            measurements[ind].header.stamp.sec = timestamp.as_secs() as i32;
            measurements[ind].header.stamp.nanosec = timestamp.subsec_nanos() as u32;

            measurements[ind].magnetic_field.x = x;
            measurements[ind].magnetic_field.y = y;
            measurements[ind].magnetic_field.z = z;

            let covariance = [0f64; 9];
            measurements[ind].magnetic_field_covariance = covariance;

        }
        // dbg!(measurements);

        message.data = measurements.try_into().unwrap();
        publisher.publish(&message);

    }
    // Clean up the threads to avoid issues
    service0_tx.send(ServiceThreadCommands::Kill).unwrap();
    service1_tx.send(ServiceThreadCommands::Kill).unwrap();

    Ok(())
}
