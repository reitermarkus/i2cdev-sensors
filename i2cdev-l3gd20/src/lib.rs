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
use i2cdev::linux::{LinuxI2CDevice,LinuxI2CError};
use byteorder::{ByteOrder, BigEndian, LittleEndian};

// Addresses

pub const L3GD20_ADDR: u16 = 0x6A;

const L3GD20_CTRL_REG1: u8 = 0x20;
const L3GD20_CTRL_REG2: u8 = 0x21;
const L3GD20_CTRL_REG3: u8 = 0x22;
const L3GD20_CTRL_REG4: u8 = 0x23;
const L3GD20_CTRL_REG5: u8 = 0x24;

const L3GD20_INCREMENT_BIT: u8 = 0x80;

const L3GD20_OUT: u8 = 0x28;

const L3GD20_WHO_AM_I: u8 = 0x0F;
const L3GD20_ID: u8 = 0b11010100;

#[derive(Debug, Copy, Clone)]
pub enum L3GD20PowerMode {
    PowerDown = 0b00000000,
    Sleep,
    Normal = 0b00001000
}

#[derive(Debug, Copy, Clone)]
pub enum L3GD20GyroscopeDataRate {
    Hz95 = 0b00,
    Hz190 = 0b01,
    Hz380 = 0b10,
    Hz760 = 0b11
}

/// Reference table 21 on data sheet
/// I believe this is the cutoff of the low pass filter
#[derive(Debug, Copy, Clone)]
pub enum L3GD20GyroscopeBandwidth {
    BW1 = 0b00,
    BW2 = 0b01,
    BW3 = 0b10,
    BW4 = 0b11
}

#[derive(Debug, Copy, Clone)]
pub enum L3GD20GyroscopeFS {
    dps250 = 0b00,
    dps500 = 0b01,
    dps2000 = 0b10
}

/// Use [data-sheet](http://www.st.com/content/ccc/resource/technical/document/datasheet/43/37/e3/06/b0/bf/48/bd/DM00036465.pdf/files/DM00036465.pdf/jcr:content/translations/en.DM00036465.pdf) to read in depth about settings
/// Specifically Registers CTRL_REG1_G - CTRL_REG5_G
#[derive(Debug, Copy, Clone)]
pub struct L3GD20GyroscopeSettings {
    /// Data measurement rate
    pub DR: L3GD20GyroscopeDataRate,
    /// Low pass filter cutoff
    pub BW: L3GD20GyroscopeBandwidth,
    /// Sleep will automatically disable '''xen''', '''yen''', '''zen'''
    pub power_mode: L3GD20PowerMode,
    /// Enable z axis readings
    pub zen: bool,
    /// Enable y axis readings
    pub yen: bool,
    /// Enable x axis readings
    pub xen: bool,
    /// Range of measurements. Lower range means more precision.
    pub sensitivity: L3GD20GyroscopeFS,
    /// Set to false if you do not want to update the buffer unless it has been read
    pub continuous_update: bool
}

/// Get Linux I2C devices at their default registers
pub fn get_linux_l3gd20_i2c_device() -> Result<LinuxI2CDevice,LinuxI2CError> {
    let gyro = try!(LinuxI2CDevice::new("/dev/i2c-1", L3GD20_ADDR));
    Ok(gyro)
}

pub struct L3GD20<T: I2CDevice + Sized> {
    pub gyroscope: T,
    g_gain: f32,
}

impl<T> L3GD20<T>
    where T: I2CDevice + Sized
{
    pub fn new(mut gyro: T, mut gyro_settings: L3GD20GyroscopeSettings) -> Result<L3GD20<T>, T::Error> {
        assert!(try!(gyro.smbus_read_byte_data(L3GD20_WHO_AM_I)) == L3GD20_ID, "L3GD20 ID didn't match for device at given I2C address.");
        let mut ctrl_reg1: u8 = 0_u8 | ((gyro_settings.DR as u8) << 6) | ((gyro_settings.BW as u8) << 4);
        match gyro_settings.power_mode {
            L3GD20PowerMode::PowerDown => {
                ctrl_reg1 |= L3GD20PowerMode::PowerDown as u8;
            },
            L3GD20PowerMode::Sleep => {
                ctrl_reg1 = (ctrl_reg1 | L3GD20PowerMode::Normal as u8) & 0b11111000;
            },
            L3GD20PowerMode::Normal => {
                ctrl_reg1 |= L3GD20PowerMode::Normal as u8;
                if gyro_settings.xen { ctrl_reg1 |= 0b001 };
                if gyro_settings.yen { ctrl_reg1 |= 0b010 };
                if gyro_settings.zen { ctrl_reg1 |= 0b100 };
            }
        }
        println!("{}", format!("{:#b}", ctrl_reg1));
        try!(gyro.smbus_write_byte_data(L3GD20_CTRL_REG1, ctrl_reg1));

        let mut ctrl_reg4: u8 = 0_u8 | ((gyro_settings.sensitivity as u8) << 4);
        if !gyro_settings.continuous_update {
            ctrl_reg4 |= 0b10000000;
        }
        try!(gyro.smbus_write_byte_data(L3GD20_CTRL_REG4, ctrl_reg4));

        // Calculate gain
        let mut g_gain: f32;

        match gyro_settings.sensitivity {
            L3GD20GyroscopeFS::dps250 => {
                g_gain = 8.75 / 1000.0;
            },
            L3GD20GyroscopeFS::dps500 => {
                g_gain = 17.50 / 1000.0;
            },
            L3GD20GyroscopeFS::dps2000 => {
                g_gain = 70.0 / 1000.0;
            }
        }

        Ok(L3GD20 {
            gyroscope: gyro,
            g_gain: g_gain,
        })
    }

    fn read_gyroscope_raw(&mut self) -> Result<(i16, i16, i16), T::Error> {
        let mut buf: [u8; 6] = [0;6];
        self.gyroscope.write(&[L3GD20_INCREMENT_BIT | L3GD20_OUT]);
        self.gyroscope.read(&mut buf);
        let x_raw = LittleEndian::read_i16(&buf[0..2]);
        let y_raw = LittleEndian::read_i16(&buf[2..4]);
        let z_raw = LittleEndian::read_i16(&buf[4..6]);
        Ok((x_raw, y_raw, z_raw))
    }
}

impl<T> Gyroscope for L3GD20<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;
    /// Returns reading in dps
    fn angular_rate_reading(&mut self) -> Result<Vec3, T::Error> {
        let (x_raw, y_raw, z_raw) = try!(self.read_gyroscope_raw());
        let angular_velocity = Vec3 {
            x: (x_raw as f32) * self.g_gain,
            y: (y_raw as f32) * self.g_gain,
            z: (y_raw as f32) * self.g_gain
        };
        Ok(angular_velocity)
    }

}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
    }
}
