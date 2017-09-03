extern crate i2cdev_lsm303dlhc;
extern crate i2csensors;

use i2cdev_lsm303dlhc::*;
use i2csensors::{Magnetometer,Accelerometer};
use std::thread;
use std::time::Duration;

#[cfg(not(any(target_os = "linux", target_os = "android")))]
fn main() {

}

#[cfg(any(target_os = "linux", target_os = "android"))]
fn main() {
    let settings = LSM303DLHCSettings {
        continuous_update: true,
        low_power: false,
        accelerometer_data_rate: LSM303DLHCAccelerometerUpdateRate::Hz400,
        accelerometer_anti_alias_filter_bandwidth: LSM303DLHCAccelerometerFilterBandwidth::Hz194,
        azen: true,
        ayen: true,
        axen: true,
        accelerometer_sensitivity: LSM303DLHCAccelerometerFS::g2,
        magnetometer_resolution: LSM303DLHCMagnetometerResolution::Low,
        magnetometer_data_rate: LSM303DLHCMagnetometerUpdateRate::Hz75,
        magnetometer_low_power_mode: false,
        magnetometer_mode: LSM303DLHCMagnetometerMode::ContinuousConversion,
        magnetometer_sensitivity: LSM303DLHCMagnetometerFS::gauss2_5
    };

    let (mut acc, mut mag) = get_linux_lsm303d_i2c_device().unwrap();

    let mut lsm303d_accel_mag = LSM303DLHC::new(acc, mag, settings).unwrap();

    lsm303d_accel_mag.magnetic_reading().unwrap();
    loop {
        let acceleration = lsm303d_accel_mag.acceleration_reading().unwrap();
        let magnetism = lsm303d_accel_mag.magnetic_reading().unwrap();
        println!("acceleration: x: {}, y: {}, z: {}, total: {}", format!("{:.*}", 2, acceleration.x), format!("{:.*}", 2, acceleration.y), format!("{:.*}", 2, acceleration.z), format!("{:.*}", 2, (acceleration.x.powi(2) + acceleration.y.powi(2) + acceleration.z.powi(2)).sqrt()));
        println!("magnetism: x: {}, y: {}, z: {}\r", format!("{:.*}", 2, magnetism.x), format!("{:.*}", 2, magnetism.y), format!("{:.*}", 2, magnetism.z));
        thread::sleep(Duration::from_millis(20));
    }
}
