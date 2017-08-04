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

use i2csensors::{Magnetometer,Accelerometer};
use i2csensors::Vec3;
use std::thread;
use std::time::Duration;
use std::error::Error;
use i2cdev::core::I2CDevice;
use byteorder::{ByteOrder, LittleEndian};
#[cfg(any(target_os = "linux", target_os = "android"))]
use i2cdev::linux::{LinuxI2CDevice,LinuxI2CError};

pub const LSM303D_I2C_ADDR: u16 = 0x1E;

const LSM303D_WHO_AM_I: u8 = 0x0F;
const LSM303D_ID: u8 = 0b01001001;

const LSM303D_INCREMENT_BIT: u8 = 0x80;
const LSM303D_OUT_MAG: u8 = 0x08;
const LSM303D_OUT_ACC: u8 = 0x28;

const LSM303D_CTRL1: u8 = 0x20;
const LSM303D_CTRL2: u8 = 0x21;
const LSM303D_CTRL5: u8 = 0x24;
const LSM303D_CTRL6: u8 = 0x25;
const LSM303D_CTRL7: u8 = 0x26;

#[derive(Debug, Copy, Clone)]
pub enum LSM303DAccelerometerUpdateRate {
    PowerDown = 0b0000,
    Hz3_125 = 0b0001,
    Hz6_25 = 0b0010,
    Hz12_5 = 0b0011,
    Hz25 = 0b0100,
    Hz50 = 0b0101,
    Hz100 = 0b0110,
    Hz200 = 0b0111,
    Hz400 = 0b1000,
    Hz800 = 0b1001,
    Hz1600 = 0b1010
}

#[derive(Debug, Copy, Clone)]
pub enum LSM303DAccelerometerFS {
    g2 = 0b000,
    g4 = 0b001,
    g6 = 0b010,
    g8 = 0b011,
    g16 = 0b100
}

#[derive(Debug, Copy, Clone)]
pub enum LSM303DMagnetometerMode {
    ContinuousConversion = 0b00,
    SingleConversion = 0b01,
    PowerDown = 0b10
}

#[derive(Debug, Copy, Clone)]
pub enum LSM303DMagnetometerResolution {
    Low = 0b00,
    MediumLow = 0b01,
    MediumHigh = 0b10,
    High = 0b11
}

#[derive(Debug, Copy, Clone)]
pub enum LSM303DMagnetometerUpdateRate {
    Hz3_125 = 0b000,
    Hz6_25 = 0b001,
    Hz12_5 = 0b010,
    Hz25 = 0b011,
    Hz50 = 0b100,
    Hz100 = 0b101
}

#[derive(Debug, Copy, Clone)]
pub enum LSM303DMagnetometerFS {
    gauss2 = 0b00,
    gauss4 = 0b01,
    gauss8 = 0b10,
    gauss12 = 0b11
}

/// Use the [data sheet](http://www.st.com/content/ccc/resource/technical/document/datasheet/1c/9e/71/05/4e/b7/4d/d1/DM00057547.pdf/files/DM00057547.pdf/jcr:content/translations/en.DM00057547.pdf) to read in depth about settings
#[derive(Debug, Copy, Clone)]
pub struct LSM303DSettings {
    /// Continuously update output registers or wait until read
    pub continuous_update: bool,
    //Accelerometer
    /// Frequency that accelerometer measurements are made
    pub accelerometer_data_rate: LSM303DAccelerometerUpdateRate,
    /// Enable accelerometer z axis
    pub azen: bool,
    /// Enable accelerometer y axis
    pub ayen: bool,
    /// Enable accelerometer x axis
    pub axen: bool,
    /// The maximum/minimum (+-) reading of acceleration (Full range)
    pub accelerometer_sensitivity: LSM303DAccelerometerFS,
    //Magnetometer
    pub magnetometer_resolution: LSM303DMagnetometerResolution,
    /// Frequency that magnetometer measurements are made
    pub magnetometer_data_rate: LSM303DMagnetometerUpdateRate,
    pub magnetometer_low_power_mode: bool,
    pub magnetometer_mode: LSM303DMagnetometerMode,
    /// The maximum/minimum (+-) reading of magnetism (Full range)
    pub magnetometer_sensitivity: LSM303DMagnetometerFS
}

/// Get Linux I2C device at L3GD20's default address
#[cfg(any(target_os = "linux", target_os = "android"))]
pub fn get_linux_lsm303d_i2c_device() -> Result<LinuxI2CDevice,LinuxI2CError> {
    let device = try!(LinuxI2CDevice::new("/dev/i2c-1", LSM303D_I2C_ADDR));
    Ok(device)
}

pub struct LSM303D<T: I2CDevice + Sized> {
    accelerometer_magnetometer: T,
    m_gain: f32,
    a_gain: f32
}

impl<T> LSM303D<T>
    where T: I2CDevice + Sized
{
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn new(mut accel_mag: T, mut accel_mag_settings: LSM303DSettings) -> Result<LSM303D<T>, T::Error> {
        assert!(try!(accel_mag.smbus_read_byte_data(LSM303D_WHO_AM_I)) == LSM303D_ID, "LSM303D ID didn't match for device at given I2C address.");

        let mut ctrl_reg1: u8 = 0_u8 | ((accel_mag_settings.accelerometer_data_rate as u8) << 4);
        if !accel_mag_settings.continuous_update {
            ctrl_reg1 |= 0b00001000;
        }
        if accel_mag_settings.axen { ctrl_reg1 |= 0b001 };
        if accel_mag_settings.ayen { ctrl_reg1 |= 0b010 };
        if accel_mag_settings.azen { ctrl_reg1 |= 0b100 };
        try!(accel_mag.smbus_write_byte_data(LSM303D_CTRL1, ctrl_reg1));

        let mut ctrl_reg2: u8 = 0_u8 | ((accel_mag_settings.accelerometer_sensitivity as u8) << 3);
        try!(accel_mag.smbus_write_byte_data(LSM303D_CTRL2, ctrl_reg2));

        let mut ctrl_reg5: u8 = 0_u8 | ((accel_mag_settings.magnetometer_resolution as u8) << 5) | ((accel_mag_settings.magnetometer_data_rate as u8) << 2); //24
        try!(accel_mag.smbus_write_byte_data(LSM303D_CTRL5, ctrl_reg5));

        let mut ctrl_reg6: u8 = 0_u8 | ((accel_mag_settings.magnetometer_sensitivity as u8) << 5);
        try!(accel_mag.smbus_write_byte_data(LSM303D_CTRL6, ctrl_reg6));

        let mut ctrl_reg7: u8 = 0_u8 | (accel_mag_settings.magnetometer_mode as u8);
        if accel_mag_settings.magnetometer_low_power_mode {
            ctrl_reg7 |= 0b00000100;
        }
        try!(accel_mag.smbus_write_byte_data(LSM303D_CTRL7, ctrl_reg7));

        let mut a_gain: f32;
        let mut m_gain: f32;

        match accel_mag_settings.magnetometer_sensitivity {
            LSM303DMagnetometerFS::gauss2 => {
                m_gain = 0.08;
            },
            LSM303DMagnetometerFS::gauss4 => {
                m_gain = 0.160;
            },
            LSM303DMagnetometerFS::gauss8 => {
                m_gain = 0.320;
            },
            LSM303DMagnetometerFS::gauss12 => {
                m_gain = 0.479;
            }
        }

        match accel_mag_settings.accelerometer_sensitivity {
            LSM303DAccelerometerFS::g2 => {
                a_gain = 0.061;
            },
            LSM303DAccelerometerFS::g4 => {
                a_gain = 0.122;
            },
            LSM303DAccelerometerFS::g6 => {
                a_gain = 0.183;
            },
            LSM303DAccelerometerFS::g8 => {
                a_gain = 0.244;
            },
            LSM303DAccelerometerFS::g16 => {
                a_gain = 0.732;
            }
        }

        Ok(LSM303D {
            accelerometer_magnetometer: accel_mag,
            m_gain: m_gain / 1000.0,
            a_gain: a_gain / 1000.0
        })
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn magnetometer_read_raw(&mut self) -> Result<(i16, i16, i16), T::Error> {
        let mut buf: [u8; 6] = [0;6];
        try!(self.accelerometer_magnetometer.write(&[LSM303D_INCREMENT_BIT | LSM303D_OUT_MAG]));
        try!(self.accelerometer_magnetometer.read(&mut buf));
        let x_raw = LittleEndian::read_i16(&buf[0..2]);
        let y_raw = LittleEndian::read_i16(&buf[2..4]);
        let z_raw = LittleEndian::read_i16(&buf[4..6]);
        Ok((x_raw, y_raw, z_raw))
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn accelerometer_read_raw(&mut self) -> Result<(i16, i16, i16), T::Error> {
        let mut buf: [u8; 6] = [0;6];
        try!(self.accelerometer_magnetometer.write(&[LSM303D_INCREMENT_BIT | LSM303D_OUT_ACC]));
        try!(self.accelerometer_magnetometer.read(&mut buf));
        let x_raw = LittleEndian::read_i16(&buf[0..2]);
        let y_raw = LittleEndian::read_i16(&buf[2..4]);
        let z_raw = LittleEndian::read_i16(&buf[4..6]);
        Ok((x_raw, y_raw, z_raw))
    }
}

impl<T> Magnetometer for LSM303D<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    fn magnetic_reading(&mut self) -> Result<Vec3, T::Error> {
        Ok(Vec3::zeros())
    }

    /// Returns reading in gauss
    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn magnetic_reading(&mut self) -> Result<Vec3, T::Error> {
        let (x_raw, y_raw, z_raw) = try!(self.magnetometer_read_raw());
        Ok(Vec3 {
            x: x_raw as f32 * self.m_gain,
            y: y_raw as f32 * self.m_gain,
            z: z_raw as f32 * self.m_gain
        })
    }
}

impl<T> Accelerometer for LSM303D<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    fn acceleration_reading(&mut self) -> Result<Vec3, T::Error> {
        Ok(Vec3::zeros())
    }

    /// Returns acceleration in gs
    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn acceleration_reading(&mut self) -> Result<Vec3, T::Error> {
        let (x_raw, y_raw, z_raw) = try!(self.accelerometer_read_raw());
        Ok(Vec3 {
            x: x_raw as f32 * self.a_gain,
            y: y_raw as f32 * self.a_gain,
            z: z_raw as f32 * self.a_gain
        })
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
    }
}
