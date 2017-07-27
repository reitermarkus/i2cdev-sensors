// Copyright 2017, Martin Deegan <mddeegan@gmail.com>
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/license/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option.  This file may not be copied, modified, or distributed
// except according to those terms.

// This module is implementing userland library to interact with
// Bosh BMP280 i2c temperature and pressure sensor.

#![allow(dead_code)]

extern crate i2cdev;
extern crate i2csensors;
extern crate byteorder;

use i2csensors::{Gyroscope};
use std::thread;
use std::time::Duration;
use std::error::Error;
use i2cdev::core::I2CDevice;
use byteorder::{ByteOrder, BigEndian};


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
    }
}
