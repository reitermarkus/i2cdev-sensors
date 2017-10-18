BMP280
====

An I2C driver for the BMP280 barometer, thermometer.

## Usage
Add the following to your `Cargo.toml`:
```toml
[dependencies]
i2csensors = "0.1.*"
i2cdev-bmp280 = "0.1.*"
```

Next, add this to your crate root:
```rust
extern crate i2cdev_bmp280;
extern crate i2csensors;
```

### Initializing and reading from a BMP280
```rust
use i2cdev_bmp280::*;
use i2csensors::{Barometer, Thermometer};

fn main() {
	let i2c_device = i2cdev_bmp280::get_linux_bmp280_i2c_device().unwrap();
	
	let settings = BMP280Settings {
                compensation: BMP280CompensationAlgorithm::B64,
                t_sb: BMP280Timing::ms0_5,
                iir_filter_coeff: BMP280FilterCoefficient::Medium,
                osrs_t: BMP280TemperatureOversampling::x1,
                osrs_p: BMP280PressureOversampling::StandardResolution,
                power_mode: BMP280PowerMode::NormalMode
    };

    let bmp280 = BMP280::new(device, settings);

    let temperatue = bmp280.temperature_celsius().unwrap();
    let pressure = bmp280.pressure_kpa().unwrap());
}
```

Settings can be adjusted according to the [datasheet](https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf).



