LSM9DS0
====

An I2C driver for the LSM9DS0 gyroscope, accelerometer, magnetometer.

## Usage
Add the following to your `Cargo.toml`:
```toml
[dependencies]
i2csensors = "0.1.*"
i2cdev-lsm9ds0 = "0.1.*"
```

Next, add this to your crate root:
```rust
extern crate i2cdev_lsm9ds0;
extern crate i2csensors;
```

### Initializing and reading from an L3GD20
```rust
use i2cdev_lsm9ds0::*;
use i2csensors::{Accelerometer,Magnetometer,Gyroscope};

fn main() {
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
}
```

Settings can be adjusted according to the [datasheet](https://cdn-shop.adafruit.com/datasheets/LSM9DS0.pdf).



