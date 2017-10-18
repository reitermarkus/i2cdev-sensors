LSM303DLHC
====

An I2C driver for the LSM303DLHC Accelerometer, Magnetometer.

## Usage
Add the following to your `Cargo.toml`:
```toml
[dependencies]
i2csensors = "0.1.*"
i2cdev-lsm303dlhc = "0.1.*"
```

Next, add this to your crate root:
```rust
extern crate i2cdev_lsm303dlhc;
extern crate i2csensors;
```

### Initializing and reading from an LSM303DLHC
```rust
use i2cdev_lsm303dlhc::*;
use i2csensors::{Magnetometer,Accelerometer};

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

    let acceleration = lsm303d_accel_mag.acceleration_reading().unwrap();
    let magnetism = lsm303d_accel_mag.magnetic_reading().unwrap();
}
```

Settings can be adjusted according to the [datasheet](http://www.st.com/content/ccc/resource/technical/document/datasheet/56/ec/ac/de/28/21/4d/48/DM00027543.pdf/files/DM00027543.pdf/jcr:content/translations/en.DM00027543.pdf).



