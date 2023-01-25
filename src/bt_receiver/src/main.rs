use std::convert::TryInto;
use std::env;
use std::fs::read;
use rclrs::Context;
use std_msgs::msg::{Float64};
use geometry_msgs::msg::{Vector3, Twist};
use serialport::{SerialPort};
use std::thread;
use std::time::Duration;
use std::mem::size_of;
use protobuf::{Message};
use anyhow::{Error, Result};


mod messages;

use messages::{Command, Response};
use crate::messages::command;
// use std_msgs::msg::Float64;


fn recv(serial: &mut Box<dyn SerialPort>, buffer: &mut [u8]) -> usize {
    let mut bytes_read = 0;
    loop {
        let available = serial.bytes_to_read().unwrap() as usize;
        if available > buffer.len() {
            bytes_read = serial.read(buffer).unwrap();
            break;
        }
    }
    return bytes_read;
}

fn compute_checksum(buffer: &[u8]) -> u16 {
    let mut sum: u16 = 0;
    for elem in buffer {
        sum = sum.wrapping_add(*elem as u16);
    }
    !sum
}

const RX_BUF_SIZE: usize = 64;

const HEADER_LEN: usize = 4;
const PREDATA_LEN: usize = HEADER_LEN + size_of::<u32>();
const CHECKSUM_LEN: usize = size_of::<u16>();

const HEADER: [u8; 4] = [0x69, 0x69, 0x20, 0x20];

fn main() -> Result<(), Error> { // Initialize Hardware

    let mut serial_tx = serialport::new("/dev/ttyUSB0", 115200)
        .timeout(Duration::from_millis(50))
        .open().expect("");

    let mut serial_rx = serial_tx.try_clone().unwrap();

    let context = Context::new(env::args())?;

    let mut node = rclrs::create_node(&context, "bt_receiver")?;


    let mut command_listener = move || {
        let drive_publisher = node.create_publisher::<Twist>("drive", rclrs::QOS_PROFILE_DEFAULT).unwrap();
        let lift_publisher = node.create_publisher::<Float64>("lift", rclrs::QOS_PROFILE_DEFAULT).unwrap();
        let mut previous_drive = Twist::default();
        let mut previous_lift = Float64::default();
        let mut drive_packet = Twist::default();
        let mut lift_packet = Float64::default();
        let mut buffer = Vec::with_capacity(RX_BUF_SIZE);
        unsafe { buffer.set_len(RX_BUF_SIZE) };
        loop {
            let mut buf_ptr: usize = 0;


            // Read header (ignore it)
            buf_ptr = HEADER_LEN;
            // Wait for header
            let mut header_ptr: usize = 0;
            loop {
                serial_rx.read(&mut buffer[header_ptr..=header_ptr]);
                header_ptr += 1;
                header_ptr &= 0b11;
                if buffer[0..4] == HEADER {
                    break;
                }
                dbg!(&buffer[0..4]);
            }

            // Recv header and data length
            recv(&mut serial_rx, &mut buffer[HEADER_LEN..PREDATA_LEN]);
            // Read data length
            let data_length = u32::from_be_bytes((&buffer[buf_ptr..PREDATA_LEN]).try_into().unwrap()) as usize;
            buf_ptr = PREDATA_LEN;

            // Recv data body
            recv(&mut serial_rx, &mut buffer[buf_ptr..buf_ptr + data_length]);
            buf_ptr += data_length;

            // Recv and read checksum
            recv(&mut serial_rx, &mut buffer[buf_ptr..buf_ptr + CHECKSUM_LEN]);
            let checksum = u16::from_be_bytes((&buffer[buf_ptr..buf_ptr + CHECKSUM_LEN]).try_into().unwrap());
            dbg!(&buffer);
            dbg! {&checksum};
            dbg! {&compute_checksum(buffer.as_slice())};

            // Verify checksum and dispatch messages
            if checksum == compute_checksum(&buffer[0..buf_ptr]) {
                let packet = &buffer[PREDATA_LEN..buf_ptr];

                let command: Command = Command::parse_from_bytes(packet).unwrap();

                let mut drive_packet = Twist::default();
                let mut lift_packet = Float64::default();

                drive_packet.linear.x = if command.teleop.drive_x == -2f64 { previous_drive.linear.x } else { command.teleop.drive_x };
                drive_packet.linear.y = if command.teleop.drive_y == -2f64 { previous_drive.linear.y } else { command.teleop.drive_y };
                lift_packet.data = if command.teleop.lift == -2f64 { previous_lift.data } else { command.teleop.lift };

                previous_drive = drive_packet.clone();
                previous_lift = lift_packet.clone();
                dbg!(&drive_packet);
                dbg!(&lift_packet);
                drive_publisher.publish(drive_packet);
                lift_publisher.publish(lift_packet);
            }

            // This is the only byte of the buffer that needs to be cleared, as it "taints" the existing header data
            // Which ensures the program will only read the next packet after a complete header is read
            // Much more efficient than clearing the entire buffer
            buffer[HEADER_LEN-1] = 0;

        }
    };
    // let thread = thread::spawn(command_listener);
    command_listener();
    loop {}

    // let _drive_subscription = node.create_subscription::<Twist, _>("drive", rclrs::QOS_PROFILE_DEFAULT, drive_event);
    // let _lift_subscription = node.create_subscription::<Float64, _>("lift", rclrs::QOS_PROFILE_DEFAULT, lift_event);
}

