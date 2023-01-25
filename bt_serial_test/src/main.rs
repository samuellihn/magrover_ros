use std::time::Duration;
use serialport::SerialPort;

const HEADER: [u8; 4] = [0x69, 0x69, 0x20, 0x20];
fn main() {
    let mut serial = serialport::new("COM6", 115200)
            .timeout(Duration::from_millis(10))
            .open().expect("Failed to open servo controller serial port.");
    loop {
        let mut line = String::new();
        println!("Enter data:");
        let len = std::io::stdin().read_line(&mut line).unwrap();
        serial.write(&HEADER);
        serial.write(&(len as u32).to_be_bytes());
        serial.write(line.as_bytes());
    }



}
