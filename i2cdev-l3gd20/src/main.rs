extern crate i2cdev_l3gd20;
extern crate i2csensors;

use i2cdev_l3gd20::*;
use i2csensors::Gyroscope;
use std::thread;
use std::time::Duration;

fn main() {
    let settings = L3GD20GyroscopeSettings {
        DR: L3GD20GyroscopeDataRate::Hz190,
        BW: L3GD20GyroscopeBandwidth::BW1,
        power_mode: L3GD20PowerMode::Normal,
        zen: true,
        yen: true,
        xen: true,
        sensitivity: L3GD20GyroscopeFS::dps500,
        continuous_update: true
    };

    let mut i2cdev = get_linux_l3gd20_i2c_device().unwrap();

    let mut l3gd20_gyro = L3GD20::new(i2cdev, settings).unwrap();

    for i in 0..400 {
        let reading = l3gd20_gyro.angular_rate_reading().unwrap();
        println!("x: {}, y: {}, z: {}", format!("{:.*}", 2, reading.x), format!("{:.*}", 2, reading.y), format!("{:.*}", 2, reading.z));
        thread::sleep(Duration::from_millis(50));
    }
}