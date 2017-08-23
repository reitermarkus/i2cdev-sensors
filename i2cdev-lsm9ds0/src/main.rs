extern crate i2cdev_lsm9ds0;
extern crate i2csensors;
extern crate i2cdev;

use i2cdev_lsm9ds0::*;
use i2csensors::{Accelerometer,Magnetometer,Gyroscope};
use std::thread;
use std::time::{Duration,Instant};
use std::io::stdin;
use std::string::String;

#[cfg(not(any(target_os = "linux", target_os = "android")))]
fn main() {

}

#[cfg(any(target_os = "linux", target_os = "android"))]
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
        continuous_update: true,
        high_pass_filter_enabled: true,
        high_pass_filter_mode: Some(LSM9DS0GyroscopeHighPassFilterMode::NormalMode),
        high_pass_filter_configuration: Some(LSM9DS0HighPassFilterCutOffConfig::HPCF_0)
    };

    let accel_mag_settings = LSM9DS0AccelerometerMagnetometerSettings {
        continuous_update: true,
        accelerometer_data_rate: LSM9DS0AccelerometerUpdateRate::Hz100,
        accelerometer_anti_alias_filter_bandwidth: LSM9DS0AccelerometerFilterBandwidth::Hz50,
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

    println!("Place sensors on a level surface and press enter");
    let mut input = String::new();
    stdin().read_line(&mut input).expect("Error");

    let (mut acc_x_calib, mut acc_y_calib, mut acc_z_calib) = (0.0, 0.0, 0.0);
    let (mut gyro_x_calib, mut gyro_y_calib, mut gyro_z_calib) = (0.0, 0.0, 0.0);
    let mut bearing_offset = 0.0;
    {
        for i in 0..50 {
            let mut linear_acc = lsm9ds0.acceleration_reading().unwrap();
            let dps = lsm9ds0.angular_rate_reading().unwrap();
            let mut magnetometer_output = lsm9ds0.magnetic_reading().unwrap();

            acc_x_calib += linear_acc.x;
            acc_y_calib += linear_acc.y;
            acc_z_calib += linear_acc.z;

            gyro_x_calib += dps.x;
            gyro_y_calib += dps.y;
            gyro_z_calib += dps.z;

            if magnetometer_output.y > 0.0 {
                bearing_offset += 90.0 - (magnetometer_output.x / magnetometer_output.y).atan() * 180.0 / std::f32::consts::PI;
            } else if magnetometer_output.y < 0.0 {
                bearing_offset += 270.0 - (magnetometer_output.x / magnetometer_output.y).atan() * 180.0 / std::f32::consts::PI;
            } else if magnetometer_output.x > 0.0 {
                bearing_offset += 180.0
            }

            thread::sleep(Duration::from_millis(50));
        }

        acc_x_calib /= 50.0;
        acc_y_calib /= 50.0;
        acc_z_calib /= 50.0;
        acc_z_calib -= 1.0;

        gyro_x_calib /= 50.0;
        gyro_y_calib /= 50.0;
        gyro_z_calib /= 50.0;

        bearing_offset /= 50.0;
        z_sum = bearing_offset;
    }

    loop {
//        println!("Starting filter.");
        let dt: f32 = last.elapsed().subsec_nanos() as f32 / 1000000000.0;
        let mut dps = lsm9ds0.angular_rate_reading().unwrap();
        dps.x -= gyro_x_calib;
        dps.y -= gyro_y_calib;
        dps.z -= gyro_z_calib;
        let (dx, dy, dz) = (dps.x * dt, dps.y * dt, dps.z * dt);
        x_sum += dx; y_sum += dy; z_sum += dz;
    //    println!("sum: x: {}, y: {}, z: {}", x_sum, y_sum, z_sum);

        let mut linear_acc = lsm9ds0.acceleration_reading().unwrap();
        linear_acc.x -= acc_x_calib;
        linear_acc.y -= acc_y_calib;
        let angle_acc_x = linear_acc.y.atan2(linear_acc.z) * 180.0 / 3.14;
        let angle_acc_y = linear_acc.x.atan2(linear_acc.z) * -180.0 / 3.14;
        //println!("linear acc: x: {}, y: {}, z: {}", format!("{:.*}", 2, linear_acc.x), format!("{:.*}", 2, linear_acc.y), format!("{:.*}", 2, linear_acc.z));
        // println!("Angular rate: x: {}, y: {}, z: {}", format!("{:.*}", 2, dps.x), format!("{:.*}", 2, dps.y), format!("{:.*}", 2, dps.z));
//        println!("Acc angle: x: {}, y: {}", format!("{:.*}", 2, angle_acc_x), format!("{:.*}", 2, angle_acc_y));

        let RADIAN_DEGREES = 180.0 / 3.14;
        let mut bearing = 0.0;
        let mut magnetometer_output = lsm9ds0.magnetic_reading().unwrap();
        // println!("mag out: {:?}", magnetometer_output);
        if magnetometer_output.y > 0.0 {
            bearing = 90.0 - (magnetometer_output.x / magnetometer_output.y).atan() * 180.0 / std::f32::consts::PI;
        } else if magnetometer_output.y < 0.0 {
            bearing = 270.0 - (magnetometer_output.x / magnetometer_output.y).atan() * 180.0 / std::f32::consts::PI;
        } else if magnetometer_output.x > 0.0 {
            bearing = 180.0
        }

        // bearing -= bearing_offset;
        // if bearing > 360.0 {
        //     bearing -= 360.0;
        // } else if bearing < 0.0 {
        //     bearing += 360.0;
        // }

        let alpha = 0.02;
        let alpha_y = 0.0;

        x_sum = x_sum * (1.0 - alpha) + angle_acc_x * alpha;
        y_sum = y_sum * (1.0 - alpha) + angle_acc_y * alpha;

        z_sum = z_sum * (1.0 - alpha_y) + bearing * alpha_y;
        if z_sum > 360.0 {
            z_sum -= 360.0;
        } else if z_sum < 0.0 {
            z_sum += 360.0;
        }

        println!("Current angle: x: {}, y: {} z: {}", format!("{:.*}", 2, x_sum), format!("{:.*}", 2, y_sum), format!("{:.*}", 2, z_sum));
        last = Instant::now();
        thread::sleep(Duration::from_millis(4));
    }
}
