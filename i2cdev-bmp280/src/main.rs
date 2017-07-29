extern crate i2cdev_bmp280;
extern crate i2csensors;
extern crate i2cdev;

use i2cdev_bmp280::*;
use i2csensors::Thermometer;
use i2csensors::Barometer;
use std::thread;
use std::time::Duration;

fn main() {
    println!("BMP280 Barometer Thermometer.");
    match i2cdevbmp280::get_linux_bmp280_i2c_device() {
        Ok(device) => {
            println!("Created new device.");
            let settings = BMP280Settings {
                compensation: BMP280CompensationAlgorithm::B64,
                t_sb: BMP280Timing::UltraFast,
                iir_filter_coeff: BMP280FilterCoefficient::Medium,
                osrs_t: BMP280TemperatureOversampling::x1,
                osrs_p: BMP280PressureOversampling::StandardResolution,
                power_mode: BMP280PowerMode::NormalMode
            };

            match BMP280::new(device, settings) {
                Ok(mut bmp280) => {
                    println!("Created new bmp280 object");

                    for i in 1..400{
                        println!("T: {}, P: {}", bmp280.temperature_celsius().unwrap(), bmp280.pressure_kpa().unwrap());
                        thread::sleep(Duration::from_millis(50));
                    }

                    thread::sleep(Duration::from_millis(500));
                },
                Err(e) => {}
            }
        },
        Err(e) => {}
    }


}

