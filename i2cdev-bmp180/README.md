BMP180
====

An I2C driver for the BMP180 barometer, thermometer.

## Usage
Add the following to your `Cargo.toml`:

```toml
[dependencies]
i2csensors = "0.1.*"
i2cdev-bmp180 = "0.1.*"
```

Next, add the following to your crate root:
```rust
extern crate i2cdev_bmp180;
extern crate i2csensors;
```