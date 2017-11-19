// Copyright 2017, Martin Deegan <mddeegan@gmail.com>
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/license/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option.  This file may not be copied, modified, or distributed
// except according to those terms.

// This module is implementing userland library to interact with
// ST L3GD20 and L3GD20H Gyroscope i2c sensors.

#![allow(dead_code)]

extern crate i2cdev;
extern crate i2csensors;
extern crate byteorder;

use i2csensors::Gyroscope;
use i2csensors::Vec3;
use std::thread;
use std::time::Duration;
use std::error::Error;
use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice,LinuxI2CError};
use byteorder::{ByteOrder, BigEndian, LittleEndian};

// Addresses

pub const L3GD20_I2C_ADDR: u16 = 0x6A;
pub const L3GD20H_I2C_ADDR: u16 = 0x6B;

const L3GD20_CTRL_REG1: u8 = 0x20;
const L3GD20_CTRL_REG2: u8 = 0x21;
const L3GD20_CTRL_REG3: u8 = 0x22;
const L3GD20_CTRL_REG4: u8 = 0x23;
const L3GD20_CTRL_REG5: u8 = 0x24;

const L3GD20_INCREMENT_BIT: u8 = 0x80;

const L3GD20_OUT: u8 = 0x28;

const L3GD20_WHO_AM_I_REGISTER: u8 = 0x0F;
const L3GD20H_WHO_AM_I: u8 = 0b11010111;
const L3GD20_WHO_AM_I: u8 = 0b11010100;

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

#[derive(Debug, Copy, Clone)]
pub enum L3GD20GyroscopeHighPassFilterMode {
    NormalModeHPRESETFILTER = 0b00,
    ReferenceSignalForFiltering = 0b01,
    NormalMode = 0b10,
    AutoresetOnInterruptEvent = 0b11
}

#[derive(Debug, Copy, Clone)]
pub enum L3GD20HighPassFilterCutOffConfig {
    HPCF_0 = 0b0000,
    HPCF_1 = 0b0001,
    HPCF_2 = 0b0010,
    HPCF_3 = 0b0011,
    HPCF_4 = 0b0100,
    HPCF_5 = 0b0101,
    HPCF_6 = 0b0110,
    HPCF_7 = 0b0111,
    HPCF_8 = 0b1000,
    HPCF_9 = 0b1001,
}

/// Use the [data sheet](http://www.st.com/content/ccc/resource/technical/document/datasheet/43/37/e3/06/b0/bf/48/bd/DM00036465.pdf/files/DM00036465.pdf/jcr:content/translations/en.DM00036465.pdf) to read in depth about settings
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
    pub continuous_update: bool,
    pub high_pass_filter_enabled: bool,
    pub high_pass_filter_mode: Option<L3GD20GyroscopeHighPassFilterMode>,
    pub high_pass_filter_configuration: Option<L3GD20HighPassFilterCutOffConfig>
}

/// Get Linux I2C device at L3GD20's default address
#[cfg(any(target_os = "linux", target_os = "android"))]
pub fn get_linux_l3gd20_i2c_device() -> Result<LinuxI2CDevice,LinuxI2CError> {
    let gyro = try!(LinuxI2CDevice::new("/dev/i2c-1", L3GD20_I2C_ADDR));
    Ok(gyro)
}

pub fn get_linux_l3gd20h_i2c_device() -> Result<LinuxI2CDevice,LinuxI2CError> {
    let gyro = try!(LinuxI2CDevice::new("/dev/i2c-1", L3GD20H_I2C_ADDR));
    Ok(gyro)
}

pub struct L3GD20<T: I2CDevice + Sized> {
    pub gyroscope: T,
    g_gain: f32,
}

impl<T> L3GD20<T>
    where T: I2CDevice + Sized
{
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn new(mut gyro: T, mut gyro_settings: L3GD20GyroscopeSettings) -> Result<L3GD20<T>, T::Error> {
        let who_am_i = try!(gyro.smbus_read_byte_data(L3GD20_WHO_AM_I_REGISTER));
        assert!(who_am_i == L3GD20_WHO_AM_I || who_am_i == L3GD20H_WHO_AM_I, "L3GD20 ID didn't match for device at given I2C address.");
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
        try!(gyro.smbus_write_byte_data(L3GD20_CTRL_REG1, ctrl_reg1));

        let mut ctrl_reg2: u8 = 0_u8;
        match gyro_settings.high_pass_filter_mode {
            Some(mode) => {
                ctrl_reg2 = ctrl_reg2 | ((mode as u8) << 4);
            },
            None => {}
        }
        match gyro_settings.high_pass_filter_configuration {
            Some(config) => {
                ctrl_reg2 = ctrl_reg2 | (config as u8);
            },
            None => {}
        }
        try!(gyro.smbus_write_byte_data(L3GD20_CTRL_REG2, ctrl_reg2));

        let mut ctrl_reg4: u8 = 0_u8 | ((gyro_settings.sensitivity as u8) << 4);
        if !gyro_settings.continuous_update {
            ctrl_reg4 |= 0b10000000;
        }
        try!(gyro.smbus_write_byte_data(L3GD20_CTRL_REG4, ctrl_reg4));

        let mut ctrl_reg5: u8 = 0_u8;
        if gyro_settings.high_pass_filter_enabled {
            ctrl_reg5 = 0b00010000;
        }
        try!(gyro.smbus_write_byte_data(L3GD20_CTRL_REG5, ctrl_reg5));

        // Calculate gain
        let mut g_gain: f32;

        match gyro_settings.sensitivity {
            L3GD20GyroscopeFS::dps250 => {
                g_gain = 8.75;
            },
            L3GD20GyroscopeFS::dps500 => {
                g_gain = 17.50;
            },
            L3GD20GyroscopeFS::dps2000 => {
                g_gain = 70.0;
            }
        }

        Ok(L3GD20 {
            gyroscope: gyro,
            g_gain: g_gain / 1000.0,
        })
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
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

    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    fn angular_rate_reading(&mut self) -> Result<Vec3, T::Error> {
        Ok(Vec3::zeros())
    }

    /// Returns reading in dps
    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn angular_rate_reading(&mut self) -> Result<Vec3, T::Error> {
        let (x_raw, y_raw, z_raw) = try!(self.read_gyroscope_raw());
        let angular_velocity = Vec3 {
            x: (x_raw as f32) * self.g_gain,
            y: (y_raw as f32) * self.g_gain,
            z: (z_raw as f32) * self.g_gain
        };
        Ok(angular_velocity)
    }

}
