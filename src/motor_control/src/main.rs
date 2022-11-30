use std::sync::{Arc, Mutex};
use lewansoul_bus_servo::ServoController;
use std::env;
use rclrs::Context;
use geometry_msgs::msg::{Vector3, Twist};
use std_msgs::msg::Float64;

use anyhow::{Error, Result};


// Motor ID
const FRONT_RIGHT: u8 = 1;
const FRONT_LEFT: u8 = 2;
const REAR_RIGHT: u8 = 3;
const REAR_LEFT: u8 = 4;
const LIFT: u8 = 5;

// Motor Coefficients
const FL_COEFF: i16 = 1;
const FR_COEFF: i16 = -1;
const RL_COEFF: i16 = 1;
const RR_COEFF: i16 = -1;
const LIFT_COEFF: i16 = -1;


fn main() -> Result<(), Error> { // Initialize Hardware
    let mut servo_controller = ServoController::new(&String::from("/dev/ttyAMA1"));

    // Set all servos to motor mode
    for i in 1..=5 {
        servo_controller.servo_or_motor_mode(i, true);
    }

    let drive_servo_controller = Arc::new(Mutex::new(servo_controller));
    let lift_servo_controller = Arc::clone(&drive_servo_controller);

    // Initialize ROS node
    let context = Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "motor_control")?;

    let drive_event = move |direction: Twist| {
        let (right_mag, left_mag) = vec3_to_tank(direction.linear);

        let right = (right_mag * 1000f64) as i16;
        let left = (left_mag * 1000f64) as i16;
        println!("{}, {}", right, left);

        let mut _controller = drive_servo_controller.lock().unwrap();
        _controller.motor(FRONT_LEFT, left * FL_COEFF);
        _controller.motor(FRONT_RIGHT, right * FR_COEFF);
        _controller.motor(REAR_LEFT, left * RL_COEFF);
        _controller.motor(REAR_RIGHT, right * RR_COEFF);
    };

    let lift_event = move |speed: Float64| {
        let speed_v = (speed.data * 1000f64) as i16;

        let mut _controller = lift_servo_controller.lock().unwrap();
        _controller.motor(LIFT, speed_v * LIFT_COEFF);
    };

    let _drive_subscription = node.create_subscription::<Twist, _>("drive", rclrs::QOS_PROFILE_DEFAULT, drive_event);
    let _lift_subscription = node.create_subscription::<Float64, _>("lift", rclrs::QOS_PROFILE_DEFAULT, lift_event);

    rclrs::spin(&node).map_err(|err| err.into())
}

// Algorithm from https://home.kendra.com/mauser/joystick.html
fn vec3_to_tank(input: Vector3) -> (f64, f64) {
    // Invert X (step 2)
    let x = -input.x;
    let y = input.y;

    // Calculate R + L (step 3)
    let v = (1f64 - x.abs()) * y + y;
    // Calculate R - L (step 4)
    let w = (1f64 - y.abs()) * x + x;
    // Calculate R and L (step 5, 6)
    let right = (v + w) / 2f64;
    let left = (v - w) / 2f64;

    (right, left)
}