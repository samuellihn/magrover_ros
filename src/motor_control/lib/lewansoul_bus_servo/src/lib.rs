mod commands;

use commands::{ServoWriteCommand, ServoReadCommand};

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}

use serialport::{SerialPort};
use std::time::Duration;


pub struct ServoController {
    serial: Box<dyn SerialPort>,
}

impl ServoController {
    pub fn new(port: &String) -> ServoController {
        let serial = serialport::new(port, 115200)
            .timeout(Duration::from_millis(10))
            .open().expect("Failed to open servo controller serial port.");
        ServoController { serial: serial }
    }
}

impl ServoController {
    fn write_command(&mut self, id: u8, cmd: ServoWriteCommand, params: &Vec<u8>) {
        let len: u8 = 3 + (params.len() as u8);
        let cmd_cast = cmd as u8;
        let mut buf: Vec<u8> = Vec::with_capacity((len + 2) as usize);
        buf.extend_from_slice(&[0x55, 0x55, id, len, cmd_cast]);
        buf.extend(params);
        let checksum = ServoController::checksum(id, len, &cmd_cast, params);
        buf.push(checksum);
        // for x in &buf {
        //     print!("{:x} ", x);
        // }
        // println!();

        self.serial.write(buf.as_slice());
    }
    fn read_command(&mut self, id: u8, cmd: ServoReadCommand, data_len: usize) -> Vec<u8> {
        let len: u8 = 3;
        let cmd_cast = cmd as u8;
        let checksum = ServoController::checksum(id, len, &cmd_cast, &vec![]);
        let mut buf: Vec<u8> = vec![0x55, 0x55, len, cmd_cast, checksum];
        self.serial.write(buf.as_slice());
        let mut read_buf: Vec<u8> = Vec::with_capacity(data_len);
        unsafe { read_buf.set_len(data_len) }
        let bytes_read = self.serial.read(&mut read_buf).unwrap();

        read_buf
    }

    #[inline]
    fn checksum(id: u8, length: u8, cmd: &u8, params: &Vec<u8>) -> u8 {
        let mut sum = id.wrapping_add(length + cmd);
        for p in params { sum = sum.wrapping_add(*p); };
        !sum
    }

    #[inline]
    fn calc_angle(angle: f32) -> u16 {
        ((angle / 240.0) * 1000.0) as u16
    }
}

impl ServoController {
    pub fn move_timed(&mut self, id: u8, angle: f32, time_ms: u16) {
        let value = ServoController::calc_angle(angle);
        let mut buf: Vec<u8> = value.to_le_bytes().to_vec();
        buf.extend_from_slice(&time_ms.to_le_bytes());
        self.write_command(id, ServoWriteCommand::MoveTime, &buf);
    }

    pub fn move_timed_batch(&mut self, body: &[(u8, f32, u16)]) {
        for a in body {
            self.move_timed(a.0, a.1, a.2);
        }
    }

    pub fn move_timed_wait(&mut self, id: u8, angle: f32, time_ms: u16) {
        let value = ServoController::calc_angle(angle);
        let mut buf: Vec<u8> = value.to_le_bytes().to_vec();
        buf.extend_from_slice(&time_ms.to_le_bytes());
        self.write_command(id, ServoWriteCommand::MoveTimeWait, &buf);
    }

    pub fn move_timed_wait_batch(&mut self, body: &[(u8, f32, u16)]) {
        for a in body {
            self.move_timed_wait(a.0, a.1, a.2);
        }
    }

    pub fn move_start(&mut self, id: u8) {
        self.write_command(id, ServoWriteCommand::MoveStart, &vec![]);
    }

    pub fn move_start_batch(&mut self, ids: &[u8]) {
        for i in ids {
            self.move_start(*i);
        }
    }

    pub fn move_stop(&mut self, id: u8) {
        self.write_command(id, ServoWriteCommand::MoveStop, &vec![]);
    }

    pub fn servo_move_stop_batch(&mut self, ids: &[u8]) {
        for i in ids {
            self.move_stop(*i);
        }
    }

    pub fn id_write(&mut self, id: u8, new_id: u8) {
        self.write_command(id, ServoWriteCommand::Id, &vec![new_id])
    }
    pub fn motor_batch(&mut self, body: &[(u8, i16)]) {
        for x in body {
            self.motor(x.0, x.1);
        }
    }

    pub fn motor(&mut self, id: u8, speed: i16) {
        let mut buf: Vec<u8> = vec![0x01, 0x00];
        buf.extend_from_slice(&speed.to_le_bytes());
        self.write_command(id, ServoWriteCommand::IsMotor, &buf);
    }

    pub fn servo_or_motor_mode(&mut self, id: u8, is_motor: bool) {
        let buf: Vec<u8> = vec![(is_motor as u8), 0x00, 0x00, 0x00];
        self.write_command(id, ServoWriteCommand::IsMotor, &buf)
    }

    pub fn servo_power(&mut self, id: u8, on: bool) {
        self.write_command(id, ServoWriteCommand::LoadUnload, &vec![id, on as u8])
    }

    pub fn get_temp(&mut self, id: u8) -> u8 {
        return self.read_command(id, ServoReadCommand::Temp, 1)[0];
    }

    pub fn read_pos(&mut self, id: u8) -> f32 {
        let buf = self.read_command(id, ServoReadCommand::Pos, 2);
        let angle_raw = u16::from_le_bytes([buf[0], buf[1]]);
        let angle: f32 = 240.0 * angle_raw as f32 / 1000.0;
        angle
    }
    pub fn motor_speed(&mut self, id: u8) -> i16 {
        let buf = self.read_command(id, ServoReadCommand::IsMotor, 4);
        let is_motor = buf[0] != 0;
        let speed = i16::from_le_bytes([buf[2], buf[3]]);
        return speed;
    }

    pub fn is_on(&mut self, id: u8) -> bool {
        let buf = self.read_command(id, ServoReadCommand::LoadUnload, 1);
        buf[0] != 0
    }
}
