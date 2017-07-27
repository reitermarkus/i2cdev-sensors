// Copyright 2017, Martin Deegan <mddeegan@gmail.com>
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/license/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option.  This file may not be copied, modified, or distributed
// except according to those terms.

// This module is implementing userland library to interact with
// ST LSM9DS0 Accelerometer, Magnetometer, Gyroscope i2c sensors.

#![allow(dead_code)]

extern crate i2cdev;
extern crate i2csensors;
extern crate byteorder;

use i2csensors::{Accelerometer,Magnetometer,Gyroscope};
use i2csensors::Vec3;
use std::thread;
use std::time::Duration;
use std::error::Error;
use i2cdev::core::I2CDevice;
use byteorder::{ByteOrder, BigEndian};

pub const LSM9DS0_ACCELEROMETER_ADDR: u16 = 0x1E;
pub const LSM9DS0_MAGNETOMETER_ADDR: u16 = 0x1E;
pub const LSM9DS0_GYROSCOPE_ADDR: u16 = 0x6A;

pub struct LSM9DS0<T: I2CDevice + Sized> {
    pub accelerometer: T,
    pub magnetometer: T,
    pub gyroscope: T
}

impl<T> LSM9DS0<T>
    where T: I2CDevice + Sized
{
    pub fn new(accel: T, mag: T, gyro: T) -> LSM9DS0<T> {
        
    }
}

impl<T> Accelerometer for LSM9DS0<T>
    where T: I2CDevice + Sized
{
    /// Returns reading in gs
    fn acceleration_reading(&mut self) -> Result<Vec3, Self::Error> {
        Vec3::zeros()
    }
}

impl<T> Magnetometer for LSM9DS0<T>
    where T: I2CDevice + Sized
{
    /// Returns reading in gauss
    fn magnetic_reading(&mut self) -> Result<Vec3, Self::Error> {
        Vec3::zeros()
    }
}

impl<T> Gyroscope for LSM9DS0<T>
    where T: I2CDevice + Sized
{
    /// Returns reading in dps
    fn angular_rate_reading(&mut self) -> Result<Vec3, Self::Error> {
        Vec3::zeros()
    }

}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
    }
}
