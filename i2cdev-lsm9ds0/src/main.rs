extern crate i2cdev_lsm9ds0;
extern crate i2csensors;
extern crate i2cdev;

use i2cdev_lsm9ds0::*;
use i2csensors::{Accelerometer,Magnetometer,Gyroscope};
use std::thread;
use std::time::{Duration,Instant};

fn main() {
    println!("LSM9DS0 Accelerometer Magnetometer Gyroscope.");
    let (gyro_dev,accel_dev) = get_default_lsm9ds0_linux_i2c_devices().unwrap();

    let gyro_settings = LSM9DS0GyroscopeSettings {
        DR: LSM9DS0GyroscopeDataRate::Hz190,
        BW: LSM9DS0GyroscopeBandwidth::BW1,
        power_mode: LSM9DS0PowerMode::Normal,
        zen: true,
        yen: true,
        xen: true,
        sensitivity: LSM9DS0GyroscopeFS::dps500,
        continuous_update: true
    };

    let accel_mag_settings = LSM9DS0AccelerometerMagnetometerSettings {
        continuous_update: true,
        accelerometer_data_rate: LSM9DS0AccelerometerUpdateRate::Hz100,
        azen: true,
        ayen: true,
        axen: true,
        accelerometer_sensitivity: LSM9DS0AccelerometerFS::g4,
        magnetometer_resolution: LSM9DS0MagnetometerResolution::Low,
        magnetometer_data_rate: LSM9DS0MagnetometerUpdateRate::Hz50,
        magnetometer_low_power_mode: false,
        magnetometer_mode: LSM9DS0MagnetometerMode::ContinuousConversion,
        magnetometer_sensitivity: LSM9DS0MagnetometerFS::gauss2
    };

    let mut lsm9ds0 = LSM9DS0::new(accel_dev, gyro_dev, gyro_settings, accel_mag_settings).unwrap();

    // A simple complementary filter
    let mut last = Instant::now();
    let (mut x_sum, mut y_sum, mut z_sum): (f32, f32, f32) = (0.0, 0.0, 0.0);
    loop {
//        println!("Starting filter.");
        let dt: f32 = last.elapsed().subsec_nanos() as f32 / 1000000000.0;
        let dps = lsm9ds0.angular_rate_reading().unwrap();
        let (dx, dy, dz) = (dps.x * dt, dps.y * dt, dps.z * dt);
        x_sum += dx; y_sum += dy; z_sum += dz;
//        println!("sum: x: {}, y: {}, z: {}", x_sum, y_sum, z_sum);

        let mut linear_acc = lsm9ds0.acceleration_reading().unwrap();
//        println!("linear acc: x: {}, y: {}, z: {}", linear_acc.x, linear_acc.y, linear_acc.z);
        let angle_acc_x = linear_acc.y.atan2(linear_acc.z) * 180.0 / 3.14;
        let angle_acc_y = -1.0 * linear_acc.x.atan2(linear_acc.z) * 180.0 / 3.14;
//        println!("Acc angle: x: {}, y: {}", angle_acc_x, angle_acc_y);

        let alpha = 0.02;

        x_sum = x_sum * (1.0 - alpha) + angle_acc_x * alpha;
        y_sum = y_sum * (1.0 - alpha) + angle_acc_y * alpha;

        println!("Current angle: x: {}, y: {}", format!("{:.*}", 2, x_sum), format!("{:.*}", 2, y_sum));
        last = Instant::now();
        thread::sleep(Duration::from_millis(50));
    }
}

