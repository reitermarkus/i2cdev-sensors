extern crate i2cdevbmp280;

use i2cdevbmp280::BMP280;

fn main() {
    let baro = BMP280::new(BMP280::get_linux_i2c_device());

    println!("temperature: {}", baro.temperature_celsius().unwrap());
    println!("Pressure: {}", baro.pressure_kpa().unwrap());
}

