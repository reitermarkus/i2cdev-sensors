extern crate i2cdev_lsm9ds0;
extern crate i2csensors;
extern crate i2cdev;

use i2cdev_lsm9ds0::*;
use i2csensors::{Accelerometer,Magnetometer,Gyroscope};
use std::thread;
use std::time::Duration;

fn main() {
    println!("LSM9DS0 Accelerometer Magnetometer Gyroscope.");
    match get_linux_lsm9ds0_i2c_devices() {
        Ok((accel_mag, gyro)) => {
            println!("Created new device.");

            let gyro_settings = LSM9DS0GyroscopeSettings {
                DR: 0b01,
                BW: 0b11,
                power_mode: LSM9DS0PowerMode::Normal,
                zen: true,
                yen: true,
                xen: true,
                filter_mode: LSM9DS0FilterMode::NormalModeReset,
                high_pass_filter_cutoff: 0b0000
            };

            let accel_settings = LSM9DS0AcclerometerMagnetometerSettings {
                acceleration_data_rate: 0b0110, // 100hz
                continuous_update: true,
                azen: true,
                ayen: true,
                axen: true,
                antialias_filter_bandwidth: 0b00,
                acceleration_full_scale_selection: 0b011,
                magnetic_resolution: LSM9DS0MagneticResolution::Low,
                magnetic_data_rate_selection: 0b110,
                magnetic_full_scale_selection: 0b00,
                acceleration_high_pass_filter: false,
                acceleration_high_pass_filter_mode: LSM9DS0HighPassFilterMode::NormalModeReset,
                magnetic_data_low_power_mode: false,
                magnetic_sensor_mode: LSM9DS0MagneticSensorMode::ContinuousConversionMode
            };

            match LSM9DS0::new(accel_mag, gyro, accel_settings, gyro_settings) {
                Ok(mut lsm9ds0) => {
                    println!("Created new LSM9DS0 object");

                    for i in 1..400{
                        thread::sleep(Duration::from_millis(50));
                    }
                },
                Err(e) => {}
            }
        },
        Err(e) => {}
    }


}

