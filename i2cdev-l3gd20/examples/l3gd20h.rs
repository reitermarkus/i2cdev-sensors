extern crate i2cdev_l3gd20;
extern crate i2csensors;

use i2cdev_l3gd20::*;
use i2csensors::{Gyroscope,Vec3};
use std::thread;
use std::time::Duration;

#[cfg(not(any(target_os = "linux", target_os = "android")))]
fn main() {

}

#[cfg(any(target_os = "linux", target_os = "android"))]
fn main() {
    let settings = L3GD20GyroscopeSettings {
        DR: L3GD20GyroscopeDataRate::Hz190,
        BW: L3GD20GyroscopeBandwidth::BW1,
        power_mode: L3GD20PowerMode::Normal,
        zen: true,
        yen: true,
        xen: true,
        sensitivity: L3GD20GyroscopeFS::dps500,
        continuous_update: true,
        high_pass_filter_enabled: true,
        high_pass_filter_mode: Some(L3GD20GyroscopeHighPassFilterMode::NormalMode),
        high_pass_filter_configuration: Some(L3GD20HighPassFilterCutOffConfig::HPCF_0)
    };

    let mut i2cdev = get_linux_l3gd20h_i2c_device().unwrap();

    let mut l3gd20_gyro = L3GD20::new(i2cdev, settings).unwrap();

    loop {
        let reading = l3gd20_gyro.angular_rate_reading().unwrap();
        println!("x: {}, y: {}, z: {}", format!("{:.*}", 2, reading.x), format!("{:.*}", 2, reading.y), format!("{:.*}", 2, reading.z));
        thread::sleep(Duration::from_millis(50));
    }
}
