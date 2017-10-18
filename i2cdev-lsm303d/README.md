LSM303D
====

An I2C driver for the LSM303D Accelerometer, Magnetometer.

## Usage
Add the following to your `Cargo.toml`:
```toml
[dependencies]
i2csensors = "0.1.*"
i2cdev-lsm303d = "0.1.*"
```

Next, add this to your crate root:
```rust
extern crate i2cdev_lsm303d;
extern crate i2csensors;
```

### Initializing and reading from an LSM303D
```rust
use i2cdev_lsm303d::*;
use i2csensors::{Magnetometer,Accelerometer};

fn main() {
    let settings = LSM303DSettings {
        continuous_update: true,
        accelerometer_data_rate: LSM303DAccelerometerUpdateRate::Hz400,
        accelerometer_anti_alias_filter_bandwidth: LSM303DAccelerometerFilterBandwidth::Hz194,
        azen: true,
        ayen: true,
        axen: true,
        accelerometer_sensitivity: LSM303DAccelerometerFS::g2,
        magnetometer_resolution: LSM303DMagnetometerResolution::Low,
        magnetometer_data_rate: LSM303DMagnetometerUpdateRate::Hz100,
        magnetometer_low_power_mode: false,
        magnetometer_mode: LSM303DMagnetometerMode::ContinuousConversion,
        magnetometer_sensitivity: LSM303DMagnetometerFS::gauss2
    };

    let mut i2cdev = get_linux_lsm303d_i2c_device().unwrap();

    let mut lsm303d_accel_mag = LSM303D::new(i2cdev, settings).unwrap();

    let acceleration = lsm303d_accel_mag.acceleration_reading().unwrap();
    let magnetism = lsm303d_accel_mag.magnetic_reading().unwrap();
}
```

Settings can be adjusted according to the [datasheet](http://www.st.com/content/ccc/resource/technical/document/datasheet/1c/9e/71/05/4e/b7/4d/d1/DM00057547.pdf/files/DM00057547.pdf/jcr:content/translations/en.DM00057547.pdf).



