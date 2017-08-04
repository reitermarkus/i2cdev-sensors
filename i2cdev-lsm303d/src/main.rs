extern crate i2cdev_lsm303d;
extern crate i2csensors;

use i2cdev_lsm303d::*;
use i2csensors::{Magnetometer,Accelerometer};
use std::thread;
use std::time::Duration;

#[cfg(not(any(target_os = "linux", target_os = "android")))]
fn main() {

}

#[cfg(any(target_os = "linux", target_os = "android"))]
fn main() {
    let settings = LSM303DSettings {
        continuous_update: true,
        accelerometer_data_rate: LSM303DAccelerometerUpdateRate::Hz100,
        azen: true,
        ayen: true,
        axen: true,
        accelerometer_sensitivity: LSM303DAccelerometerFS::g4,
        magnetometer_resolution: LSM303DMagnetometerResolution::Low,
        magnetometer_data_rate: LSM303DMagnetometerUpdateRate::Hz100,
        magnetometer_low_power_mode: false,
        magnetometer_mode: LSM303DMagnetometerMode::ContinuousConversion,
        magnetometer_sensitivity: LSM303DMagnetometerFS::gauss2
    };

    let mut i2cdev = get_linux_lsm303d_i2c_device().unwrap();

    let mut lsm303d_accel_mag = LSM303D::new(i2cdev, settings).unwrap();

    lsm303d_accel_mag.magnetic_reading().unwrap();
    for i in 0..400 {
        let acceleration = lsm303d_accel_mag.acceleration_reading().unwrap();
        let magnetism = lsm303d_accel_mag.magnetic_reading().unwrap();
        println!("acceleration: x: {}, y: {}, z: {}", format!("{:.*}", 2, acceleration.x), format!("{:.*}", 2, acceleration.y), format!("{:.*}", 2, acceleration.z));
        println!("magnetism: x: {}, y: {}, z: {}", format!("{:.*}", 2, magnetism.x), format!("{:.*}", 2, magnetism.y), format!("{:.*}", 2, magnetism.z));
        thread::sleep(Duration::from_millis(50));
    }
}
