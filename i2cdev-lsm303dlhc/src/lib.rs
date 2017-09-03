// Copyright 2017, Martin Deegan <mddeegan@gmail.com>
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/license/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option.  This file may not be copied, modified, or distributed
// except according to those terms.

// This module is implementing userland library to interact with
// lsm303dlhc accelerometer.

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
use byteorder::{ByteOrder, LittleEndian, BigEndian};
#[cfg(any(target_os = "linux", target_os = "android"))]
use i2cdev::linux::{LinuxI2CDevice,LinuxI2CError};

pub const LSM303DLHC_I2C_MAG_ADDR: u16 = 0x1E;
pub const LSM303DLHC_I2C_ACC_ADDR: u16 = 0x19;

const LSM303DLHC_INCREMENT_BIT: u8 = 0x80;
const LSM303DLHC_OUT_MAG: u8 = 0x03;
const LSM303DLHC_OUT_ACC: u8 = 0x28;

const LSM303DLHC_CTRL_REG1_A: u8 = 0x20;
const LSM303DLHC_CTRL_REG2_A: u8 = 0x21;
const LSM303DLHC_CTRL_REG4_A: u8 = 0x23;

const LSM303DLHC_CRA_REG_M: u8 = 0x00;
const LSM303DLHC_CRB_REG_M: u8 = 0x01;
const LSM303DLHC_MR_REG_M: u8 = 0x02;


#[derive(Debug, Copy, Clone)]
pub enum LSM303DLHCAccelerometerUpdateRate {
    PowerDown = 0b0000,
    Hz1 = 0b0001,
    Hz10 = 0b0010,
    Hz25 = 0b0011,
    Hz50 = 0b0100,
    Hz100 = 0b0101,
    Hz200 = 0b0110,
    Hz400 = 0b0111,
    Hz1620_LowPower = 0b1000,
    Hz1344_Normal_Hz5376_LowPower = 0b1001
}

#[derive(Debug, Copy, Clone)]
pub enum LSM303DLHCAccelerometerFilterBandwidth {
    Hz773 = 0b00,
    Hz194 = 0b01,
    Hz362 = 0b10,
    Hz50 = 0b11
}

#[derive(Debug, Copy, Clone)]
pub enum LSM303DLHCAccelerometerFS {
    g2 = 0b00,
    g4 = 0b01,
    g8 = 0b10,
    g16 = 0b011
}

#[derive(Debug, Copy, Clone)]
pub enum LSM303DLHCMagnetometerMode {
    ContinuousConversion = 0b00,
    SingleConversion = 0b01,
    PowerDown = 0b10
}

#[derive(Debug, Copy, Clone)]
pub enum LSM303DLHCMagnetometerResolution {
    Low = 0b00,
    MediumLow = 0b01,
    MediumHigh = 0b10,
    High = 0b11
}

#[derive(Debug, Copy, Clone)]
pub enum LSM303DLHCMagnetometerUpdateRate {
    Hz0_75 = 0b000,
    Hz1_5 = 0b001,
    Hz3_0 = 0b010,
    Hz7_5 = 0b011,
    Hz15 = 0b100,
    Hz30 = 0b101,
    Hz75 = 0b110,
    Hz220 = 0b111
}

#[derive(Debug, Copy, Clone)]
pub enum LSM303DLHCMagnetometerFS {
    gauss1_3 = 0b001,
    gauss1_9 = 0b010,
    gauss2_5 = 0b011,
    gauss4_0 = 0b100,
    gauss4_7 = 0b101,
    gauss5_6 = 0b110,
    gauss8_1 = 0b111
}

/// Use the [data sheet](http://www.st.com/content/ccc/resource/technical/document/datasheet/56/ec/ac/de/28/21/4d/48/DM00027543.pdf/files/DM00027543.pdf/jcr:content/translations/en.DM00027543.pdf) to read in depth about settings
#[derive(Debug, Copy, Clone)]
pub struct LSM303DLHCSettings {
    /// Continuously update output registers or wait until read
    pub continuous_update: bool,
    /// Low power mode.
    pub low_power: bool,
    //Accelerometer
    /// Frequency that accelerometer measurements are made
    pub accelerometer_data_rate: LSM303DLHCAccelerometerUpdateRate,
    pub accelerometer_anti_alias_filter_bandwidth: LSM303DLHCAccelerometerFilterBandwidth,
    /// Enable accelerometer z axis
    pub azen: bool,
    /// Enable accelerometer y axis
    pub ayen: bool,
    /// Enable accelerometer x axis
    pub axen: bool,
    /// The maximum/minimum (+-) reading of acceleration (Full range)
    pub accelerometer_sensitivity: LSM303DLHCAccelerometerFS,
    //Magnetometer
    pub magnetometer_resolution: LSM303DLHCMagnetometerResolution,
    /// Frequency that magnetometer measurements are made
    pub magnetometer_data_rate: LSM303DLHCMagnetometerUpdateRate,
    pub magnetometer_low_power_mode: bool,
    pub magnetometer_mode: LSM303DLHCMagnetometerMode,
    /// The maximum/minimum (+-) reading of magnetism (Full range)
    pub magnetometer_sensitivity: LSM303DLHCMagnetometerFS
}

/// Get Linux I2C device at L3GD20's default address
#[cfg(any(target_os = "linux", target_os = "android"))]
pub fn get_linux_lsm303d_i2c_device() -> Result<(LinuxI2CDevice,LinuxI2CDevice),LinuxI2CError> {
    let accelerometer = try!(LinuxI2CDevice::new("/dev/i2c-1", LSM303DLHC_I2C_ACC_ADDR));
    let magnometer = try!(LinuxI2CDevice::new("/dev/i2c-1", LSM303DLHC_I2C_MAG_ADDR));
    Ok((accelerometer, magnometer))
}

pub struct LSM303DLHC<T: I2CDevice + Sized> {
    accelerometer: T,
    magnometer: T,
    m_gain: f32,
    a_gain: f32
}

impl<T> LSM303DLHC<T>
    where T: I2CDevice + Sized
{
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn new(mut acc: T, mut mag: T, mut accel_mag_settings: LSM303DLHCSettings) -> Result<LSM303DLHC<T>, T::Error> {
        let mut ctrl_reg1: u8 = 0_u8 | ((accel_mag_settings.accelerometer_data_rate as u8) << 4);
        if accel_mag_settings.low_power { ctrl_reg1 |= 0b1000 };
        if accel_mag_settings.axen { ctrl_reg1 |= 0b001 };
        if accel_mag_settings.ayen { ctrl_reg1 |= 0b010 };
        if accel_mag_settings.azen { ctrl_reg1 |= 0b100 };
        try!(acc.smbus_write_byte_data(LSM303DLHC_CTRL_REG1_A, ctrl_reg1));
        
        let mut ctrl_reg4: u8 = 0_u8; 
        if !accel_mag_settings.low_power { ctrl_reg4 |= 0b1000 };
        if !accel_mag_settings.continuous_update { ctrl_reg4 |= 0b10000000 };
        ctrl_reg4 |= (accel_mag_settings.accelerometer_sensitivity as u8) << 4;
        try!(acc.smbus_write_byte_data(LSM303DLHC_CTRL_REG4_A, ctrl_reg4));

        let m_gain = match accel_mag_settings.magnetometer_sensitivity {
            LSM303DLHCMagnetometerFS::gauss1_3 => 1.0 / 1100.0,
            LSM303DLHCMagnetometerFS::gauss1_9 => 1.0 / 855.0,
            LSM303DLHCMagnetometerFS::gauss2_5 => 1.0 / 670.0,
            LSM303DLHCMagnetometerFS::gauss4_0 => 1.0 / 450.0,
            LSM303DLHCMagnetometerFS::gauss4_7 => 1.0 / 400.0,
            LSM303DLHCMagnetometerFS::gauss5_6 => 1.0 / 330.0,
            LSM303DLHCMagnetometerFS::gauss8_1 => 1.0 / 230.0
        };

        let ctrl_crb_reg_m = (accel_mag_settings.magnetometer_sensitivity as u8) << 5;
        try!(mag.smbus_write_byte_data(LSM303DLHC_CRB_REG_M, ctrl_crb_reg_m));

        let ctrl_mr_reg_m = accel_mag_settings.magnetometer_mode as u8;
        try!(mag.smbus_write_byte_data(LSM303DLHC_MR_REG_M, ctrl_mr_reg_m));

        let ctrl_cra_reg_m = (accel_mag_settings.magnetometer_data_rate as u8) << 2;
        try!(mag.smbus_write_byte_data(LSM303DLHC_CRA_REG_M, ctrl_cra_reg_m));

        let a_gain: f32 = match accel_mag_settings.accelerometer_sensitivity {
            LSM303DLHCAccelerometerFS::g2 => {
                0.061
            },
            LSM303DLHCAccelerometerFS::g4 => {
                0.122
            },
            LSM303DLHCAccelerometerFS::g8 => {
                0.244
            },
            LSM303DLHCAccelerometerFS::g16 => {
                0.732
            }
        };

        Ok(LSM303DLHC {
            accelerometer: acc,
            magnometer: mag,
            m_gain: m_gain,
            a_gain: a_gain / 1000.0
        })
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn magnetometer_read_raw(&mut self) -> Result<(i16, i16, i16), T::Error> {
        let mut buf: [u8; 6] = [0;6];
        try!(self.magnometer.write(&[LSM303DLHC_INCREMENT_BIT | LSM303DLHC_OUT_MAG]));
        try!(self.magnometer.read(&mut buf));
        let x_raw = BigEndian::read_i16(&buf[0..2]);
        let y_raw = BigEndian::read_i16(&buf[2..4]);
        let z_raw = BigEndian::read_i16(&buf[4..6]);
        Ok((x_raw, y_raw, z_raw))
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn accelerometer_read_raw(&mut self) -> Result<(i16, i16, i16), T::Error> {
        let mut buf: [u8; 6] = [0;6];
        try!(self.accelerometer.write(&[LSM303DLHC_INCREMENT_BIT | LSM303DLHC_OUT_ACC]));
        try!(self.accelerometer.read(&mut buf));
        println!("buf: {:?}", buf);
        let x_raw = LittleEndian::read_i16(&buf[0..2]);
        let y_raw = LittleEndian::read_i16(&buf[2..4]);
        let z_raw = LittleEndian::read_i16(&buf[4..6]);
        Ok((x_raw, y_raw, z_raw))
    }
}

impl<T> Magnetometer for LSM303DLHC<T>
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

impl<T> Accelerometer for LSM303DLHC<T>
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
