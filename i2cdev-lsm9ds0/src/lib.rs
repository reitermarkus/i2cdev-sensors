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

pub const LSM9DS0_ACCELEROMETER_MAGNETOMETER_ADDR: u16 = 0x1E;
pub const LSM9DS0_GYROSCOPE_ADDR: u16 = 0x6A;

const LSM9DS0_OUT_G: u8 = 0x28;
const LSM9DS0_OUT_M: u8 = 0x08;
const LSM9DS0_OUT_A: u8 = 0x28;



pub enum LSM9DS0FilterMode {
    NormalModeReset = 0b00000000,
    ReferenceSignal = 0b00010000,
    NormalMode = 0b00100000,
    AutoReset = 0b00110000
}

pub enum LSM9DS0PowerMode {
    PowerDown = 0b00000000,
    Sleep,
    Normal = 0b00001000
}

pub enum LSM9DS0GyroscopeDataRate {
    Hz95 = 0b00,
    Hz190 = 0b01,
    Hz380 = 0b10,
    Hz760 = 0b11
}

/// Reference table 21 on data sheet
/// I believe this is the cutoff of the low pass filter
pub enum LSM9DS0GyroscopeBandwidth {
    BW1 = 0b00,
    BW2 = 0b01,
    BW3 = 0b10,
    BW4 = 0b11
}

pub enum LSM9DS0GyroscopeFS {
    dps250 = 0b00,
    dps500 = 0b01,
    dps2000 = 0b10
}

pub enum LSM9DS0MagnetometerFS {
    gauss2 = 0b00,
    gauss4 = 0b01,
    gauss8 = 0b10,
    gauss12 = 0b11
}

pub enum LSM9DS0AccelerometerFS {
    g2 = 0b000,
    g4 = 0b001,
    g6 = 0b010,
    g8 = 0b011,
    g12 = 0b100
}

/// http://www.st.com/content/ccc/resource/technical/document/datasheet/ab/2a/3b/45/f0/92/41/73/DM00087365.pdf/files/DM00087365.pdf/jcr:content/translations/en.DM00087365.pdf
/// Use data-sheet to read in depth about settings
/// Specifically Registers CTRL_REG1_G - CTRL_REG5_G
/// Leave settings as 0 if you don't know
pub struct LSM9DS0GyroscopeSettings {
    pub DR: LSM9DS0GyroscopeDataRate,
    pub BW: LSM9DS0GyroscopeBandwidth,
    pub power_mode: LSM9DS0PowerMode,
    /// Enable z axis readings
    pub zen: bool,
    /// Enable y axis readings
    pub yen: bool,
    /// Enable x axis readings
    pub xen: bool,
    pub sensitivity: LSM9DS0GyroscopeFS,
    pub continuous_update: bool
}

pub struct LSM9DS0AcclerometerSettings {
    pub sensitivity: LSM9DS0AccelerometerFS
}

/// http://www.st.com/content/ccc/resource/technical/document/datasheet/ab/2a/3b/45/f0/92/41/73/DM00087365.pdf/files/DM00087365.pdf/jcr:content/translations/en.DM00087365.pdf
/// Use data-sheet to read in depth about settings.
/// Specifically Registers CTRL_REG0_XM - CTRL_REG7_XM
/// Leave settings as 0 if you don't know
pub struct LSM9DS0MagnetometerSettings {
    pub sensitivity: LSM9DS0MagnetometerFS
}

/// Get Linux I2C devices at their default registers
pub fn get_linux_lsm9ds0_i2c_devices() -> Result<(LinuxI2CDevice,LinuxI2CDevice),LinuxI2CError> {
    let accel_mag = try!(LinuxI2CDevice::new("/dev/i2c-1", LSM9DS0_ACCELEROMETER_MAGNETOMETER_ADDR));
    let gyro = try!(LinuxI2CDevice::new("/dev/i2c-1", LSM9DS0_GYROSCOPE_ADDR));
    Ok((accel_mag, gyro))
}

pub struct LSM9DS0<T: I2CDevice + Sized> {
    pub accelerometer_magnetometer: T,
    pub gyroscope: T,
    g_gain: f32,
    a_gain: f32,
    m_gain: f32
}

impl<T> LSM9DS0<T>
    where T: I2CDevice + Sized
{
    pub fn new(mut accel_mag: T, mut gyro: T,
               mut accel_settings: LSM9DS0AcclerometerSettings,
               mut mag_settings: LSM9DS0MagnetometerSettings,
               mut gyro_settings: LSM9DS0GyroscopeSettings) -> Result<LSM9DS0<T>, T::Error>
    {
//        if !(try!(accel_mag.smbus_read_byte_data(0x0F)) == 0b01001001 && try!(gyro.smbus_read_byte_data(0x0F)) == 0b11010100) {
//            panic!("LSM9DSO: At least one on chip identifier does not match.");
//        }
//
//        accel_mag_settings.assert_valid().unwrap();
//        gyro_settings.assert_valid().unwrap();
//
//        // Set accelerometer magnetometer settings
//        let mut accel_ctrl_reg_1: u8 = 0_u8 | (accel_mag_settings.acceleration_data_rate << 4);
//        if !accel_mag_settings.continuous_update {
//            accel_ctrl_reg_1 |= 0b00001000;
//        }
//        if accel_mag_settings.azen { accel_ctrl_reg_1 |= 0b00000100; }
//        if accel_mag_settings.ayen { accel_ctrl_reg_1 |= 0b00000010; }
//        if accel_mag_settings.axen { accel_ctrl_reg_1 |= 0b00000001; }
//        try!(accel_mag.smbus_write_byte_data(0x20, accel_ctrl_reg_1));
//
//        let mut accel_ctrl_reg_2: u8 = try!(accel_mag.smbus_read_byte_data(0x21)) & 0b00000111;
//        accel_ctrl_reg_2 |= (accel_mag_settings.antialias_filter_bandwidth << 6) | (accel_mag_settings.acceleration_full_scale_selection << 3);
//        try!(accel_mag.smbus_write_byte_data(0x21, accel_ctrl_reg_2));
//
//        let mut accel_ctrl_reg_5: u8 = try!(accel_mag.smbus_read_byte_data(0x24)) & 0b10000011;
//        accel_ctrl_reg_5 |= (accel_mag_settings.magnetic_resolution as u8) | (accel_mag_settings.magnetic_data_rate_selection << 2);
//        try!(accel_mag.smbus_write_byte_data(0x24, accel_ctrl_reg_5));
//
//        let mut accel_ctrl_reg_6: u8 = 0_u8;
//        accel_ctrl_reg_6 |= accel_mag_settings.magnetic_full_scale_selection << 5;
//        try!(accel_mag.smbus_write_byte_data(0x25, accel_ctrl_reg_6));
//
//        let mut accel_ctrl_reg_7: u8 = 0_u8;
//        accel_ctrl_reg_7 |= (accel_mag_settings.acceleration_high_pass_filter_mode as u8) | (accel_mag_settings.magnetic_sensor_mode as u8);
//        if accel_mag_settings.acceleration_high_pass_filter {
//            accel_ctrl_reg_7 |= 0b00100000;
//        }
//        if accel_mag_settings.magnetic_data_low_power_mode {
//            accel_ctrl_reg_7 |= 0b00000100;
//        }
//        try!(accel_mag.smbus_write_byte_data(0x26, accel_ctrl_reg_7));
//
//        // Set gyroscope settings
//        let mut gyro_ctrl_reg_1: u8 = 0_u8 | gyro_settings.DR << 6 | gyro_settings.BW << 4;
//        match gyro_settings.power_mode {
//            LSM9DS0PowerMode::PowerDown => {},
//            LSM9DS0PowerMode::Sleep => {
//                gyro_ctrl_reg_1 |= (LSM9DS0PowerMode::Normal as u8);
//            },
//            LSM9DS0PowerMode::Normal => {
//                gyro_ctrl_reg_1 |= (LSM9DS0PowerMode::Normal as u8);
//                if gyro_settings.zen { gyro_ctrl_reg_1 |= 0b00000100; }
//                if gyro_settings.yen { gyro_ctrl_reg_1 |= 0b00000010; }
//                if gyro_settings.xen { gyro_ctrl_reg_1 |= 0b00000001; }
//            }
//        }
//        try!(gyro.smbus_write_byte_data(0x20, gyro_ctrl_reg_1));
//
//        let mut gyro_ctrl_reg_2: u8 = 0_u8 | (gyro_settings.filter_mode as u8) | gyro_settings.high_pass_filter_cutoff;
//        gyro_ctrl_reg_2 &= 0b00111111;
//        try!(gyro.smbus_write_byte_data(0x21, gyro_ctrl_reg_2));

        let mut g_gain: f32;
        let mut a_gain: f32;
        let mut m_gain: f32;

        match gyro_settings.sensitivity {
            LSM9DS0GyroscopeFS::dps250 => {
                g_gain = 8.75 / 1000.0;
            },
            LSM9DS0GyroscopeFS::dps500 => {
                g_gain = 17.50 / 1000.0;
            },
            LSM9DS0GyroscopeFS::dps2000 => {
                g_gain = 70.0 / 1000.0;
            }
        }

        match accel_settings.sensitivity {
            LSM9DS0AccelerometerFS::g2 => {
                a_gain = 0.061;
            },
            LSM9DS0AccelerometerFS::g4 => {
                a_gain = 0.122;
            },
            LSM9DS0AccelerometerFS::g6 => {
                a_gain = 0.183;
            },
            LSM9DS0AccelerometerFS::g8 => {
                a_gain = 0.244;
            },
            LSM9DS0AccelerometerFS::g16 => {
                a_gain = 0.732;
            },
        }

        match mag_settings {
            LSM9DS0MagnetometerFS::gauss2 => {
                m_gain = 0.08;
            },
            LSM9DS0MagnetometerFS::gauss4 => {
                m_gain = 0.16;
            },
            LSM9DS0MagnetometerFS::gauss8 => {
                m_gain = 0.32;
            },
            LSM9DS0MagnetometerFS::gauss12 => {
                m_gain = 0.48;
            }
        }

        Ok(LSM9DS0 {
            accelerometer_magnetometer: accel_mag,
            gyroscope: gyro,
            g_gain: g_gain,
            a_gain: a_gain,
            m_gain: m_gain
        })
    }

    fn read_gyroscope_raw(&mut self) -> Result<(i16, i16, i16), T::Error>{
        let buf = try!(self.gyroscope.smbus_read_i2c_block_data(LSM9DS0_OUT_G, 6));
        let x_raw = LittleEndian::read_i16(&[buf[0..2]]);
        let y_raw = LittleEndian::read_i16(&[buf[0..2]]);
        let z_raw = LittleEndian::read_i16(&[buf[0..2]]);
        (x_raw, y_raw, z_raw)
    }

    fn read_magnetometer_raw(&mut self) -> Result<(i16,i16,i16), T::Error> {
        let buf = try!(self.accelerometer_magnetometer.smbus_read_i2c_block_data(LSM9DS0_OUT_G, 6));
        let x_raw = LittleEndian::read_i16(&[buf[0..2]]);
        let y_raw = LittleEndian::read_i16(&[buf[0..2]]);
        let z_raw = LittleEndian::read_i16(&[buf[0..2]]);
        (x_raw, y_raw, z_raw)
    }

    fn read_accelerometer_raw(&mut self) -> Result<(i16, i16, i16), T::Error> {
        let buf = try!(self.accelerometer_magnetometer.smbus_read_i2c_block_data(LSM9DS0_OUT_G, 6));
        let x_raw = LittleEndian::read_i16(&[buf[0..2]]);
        let y_raw = LittleEndian::read_i16(&[buf[0..2]]);
        let z_raw = LittleEndian::read_i16(&[buf[0..2]]);
        (x_raw, y_raw, z_raw)
    }
}

impl<T> Accelerometer for LSM9DS0<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    /// Returns reading in gs
    fn acceleration_reading(&mut self) -> Result<Vec3, T::Error> {
        let (x_raw, y_raw, z_raw) = try!(self.read_accelerometer_raw());
        let acceleration = Vec3 {
            x: (x_raw as f32),
            y: (y_raw as f32),
            z: (y_raw as f32)
        };

        Ok(acceleration)
    }
}

impl<T> Magnetometer for LSM9DS0<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    /// Returns reading in gauss
    fn magnetic_reading(&mut self) -> Result<Vec3, T::Error> {
        let (x_raw, y_raw, z_raw) = try!(self.read_magnetometer_raw());
        let magnetic_reading = Vec3 {
            x: (x_raw as f32),
            y: (y_raw as f32),
            z: (y_raw as f32)
        };

        Ok(magnetic_reading)
    }
}

impl<T> Gyroscope for LSM9DS0<T>
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
