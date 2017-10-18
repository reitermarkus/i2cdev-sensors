L3GD20
====

An I2C driver for the L3GD20 gyroscope.

## Usage
Add the following to your `Cargo.toml`:
```toml
[dependencies]
i2csensors = "0.1.*"
i2cdev-l3gd20 = "0.1.*"
```

Next, add this to your crate root:
```rust
extern crate i2cdev_l3gd20;
extern crate i2csensors;
```

### Initializing and reading from an L3GD20
```rust
use i2cdev_l3gd20::*;
use i2csensors::{Gyroscope,Vec3};

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

    let mut i2cdev = get_linux_l3gd20_i2c_device().unwrap();

    let mut l3gd20_gyro = L3GD20::new(i2cdev, settings).unwrap();

    let angular_rate = l3gd20_gyro.angular_rate_reading().unwrap();
}
```

Settings can be adjusted according to the [datasheet](https://www.pololu.com/file/0J563/L3GD20.pdf).



