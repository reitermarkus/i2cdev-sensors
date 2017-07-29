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

pub enum LSM9DS0PowerMode {
    PowerDown = 0b00000000,
    Sleep,
    Normal = 0b00001000
}

pub enum LSM9DS0FilterMode {
    NormalModeReset = 0b00000000,
    ReferenceSignal = 0b00010000,
    NormalMode = 0b00100000,
    AutoReset = 0b00110000
}

/// http://www.st.com/content/ccc/resource/technical/document/datasheet/ab/2a/3b/45/f0/92/41/73/DM00087365.pdf/files/DM00087365.pdf/jcr:content/translations/en.DM00087365.pdf
/// Use data-sheet to read in depth about settings
/// Specifically Registers CTRL_REG1_G - CTRL_REG5_G
/// Leave settings as 0 if you don't know
pub struct LSM9DS0GyroscopeSettings {
    /// 2 bits 0bXX
    pub DR: u8,
    /// 2 bits 0bXX
    pub BW: u8,
    pub power_mode: LSM9DS0PowerMode,
    /// Enable z axis readings
    pub zen: bool,
    /// Enable y axis readings
    pub yen: bool,
    /// Enable x axis readings
    pub xen: bool,
    pub filter_mode: LSM9DS0FilterMode,
    /// 4 bits 0bXXXX
    pub high_pass_filter_cutoff: u8,
}

impl LSM9DS0GyroscopeSettings {
    pub fn assert_valid(&mut self) -> Result<(),&str> {
        if self.DR > 3 || self.BW > 3 || self.high_pass_filter_cutoff > 15 {
            return Err("Invalid gyroscope settings.");
        }

        Ok(())
    }
}

pub enum LSM9DS0MagneticResolution {
    Low = 0b00000000,
    Medium = 0b00100000,
    High = 0b01000000,
    VeryHigh = 0b01100000
}

pub enum LSM9DS0HighPassFilterMode {
    NormalModeReset = 0b00000000,
    ReferenceSignal = 0b01000000,
    NormalMode = 0b10000000,
    AutoReset = 0b11000000
}

pub enum LSM9DS0MagneticSensorMode {
    ContinuousConversionMode = 0b00000000,
    SingleConversionMode = 0b00000001,
    PowerDownMode = 0b00000010
}

/// http://www.st.com/content/ccc/resource/technical/document/datasheet/ab/2a/3b/45/f0/92/41/73/DM00087365.pdf/files/DM00087365.pdf/jcr:content/translations/en.DM00087365.pdf
/// Use data-sheet to read in depth about settings.
/// Specifically Registers CTRL_REG0_XM - CTRL_REG7_XM
/// Leave settings as 0 if you don't know
pub struct LSM9DS0AcclerometerMagnetometerSettings {
    /// 4 bits 0bXXXX
    pub acceleration_data_rate: u8,
    /// Continuously update or
    pub continuous_update: bool,
    /// Enable z axis acceleration readings
    pub azen: bool,
    /// Enable y axis acceleration readings
    pub ayen: bool,
    /// Enable x axis acceleration readings
    pub axen: bool,
    /// 2 bits 0bXX
    pub antialias_filter_bandwidth: u8,
    /// 3 bits 0bXXX
    pub acceleration_full_scale_selection: u8,
    pub magnetic_resolution: LSM9DS0MagneticResolution,
    /// 3 bits 0bXXX. Default value: 0b110
    pub magnetic_data_rate_selection: u8,
    /// 2 bits 0bXXX.
    pub magnetic_full_scale_selection: u8,
    pub acceleration_high_pass_filter: bool,
    pub acceleration_high_pass_filter_mode: LSM9DS0HighPassFilterMode,
    pub magnetic_data_low_power_mode: bool,
    pub magnetic_sensor_mode: LSM9DS0MagneticSensorMode

}

impl LSM9DS0AcclerometerMagnetometerSettings {
    pub fn assert_valid(&mut self) -> Result<(),&str> {
        if self.acceleration_data_rate > 15 || self. antialias_filter_bandwidth > 3 ||
            self.acceleration_full_scale_selection > 7 || self.magnetic_data_rate_selection > 7 {
            return Err("Invalid Accelerometer Magnetometer settings.");
        }
        Ok(())
    }
}

/// Get Linux I2C devices at their default registers
pub fn get_linux_lsm9ds0_i2c_devices() -> Result<(LinuxI2CDevice,LinuxI2CDevice),LinuxI2CError> {
    let accel_mag = try!(LinuxI2CDevice::new("/dev/i2c-1", LSM9DS0_ACCELEROMETER_MAGNETOMETER_ADDR));
    let gyro = try!(LinuxI2CDevice::new("/dev/i2c-1", LSM9DS0_GYROSCOPE_ADDR));
    Ok((accel_mag, gyro))
}

pub struct LSM9DS0<T: I2CDevice + Sized> {
    pub accelerometer_magnetometer: T,
    pub gyroscope: T
}

impl<T> LSM9DS0<T>
    where T: I2CDevice + Sized
{
    pub fn new(mut accel_mag: T, mut gyro: T,
               mut accel_mag_settings: LSM9DS0AcclerometerMagnetometerSettings,
               mut gyro_settings: LSM9DS0GyroscopeSettings) -> Result<LSM9DS0<T>, T::Error>
    {
        if !(try!(accel_mag.smbus_read_byte_data(0x0F)) == 0b01001001 && try!(gyro.smbus_read_byte_data(0x0F)) == 0b11010100) {
            panic!("LSM9DSO: At least one on chip identifier does not match.");
        }

        accel_mag_settings.assert_valid().unwrap();
        gyro_settings.assert_valid().unwrap();

        // Set accelerometer magnetometer settings
        let mut accel_ctrl_reg_1: u8 = 0_u8 | (accel_mag_settings.acceleration_data_rate << 4);
        if !accel_mag_settings.continuous_update {
            accel_ctrl_reg_1 |= 0b00001000;
        }
        if accel_mag_settings.azen { accel_ctrl_reg_1 |= 0b00000100; }
        if accel_mag_settings.ayen { accel_ctrl_reg_1 |= 0b00000010; }
        if accel_mag_settings.axen { accel_ctrl_reg_1 |= 0b00000001; }
        try!(accel_mag.smbus_write_byte_data(0x20, accel_ctrl_reg_1));

        let mut accel_ctrl_reg_2: u8 = try!(accel_mag.smbus_read_byte_data(0x21)) & 0b00000111;
        accel_ctrl_reg_2 |= (accel_mag_settings.antialias_filter_bandwidth << 6) | (accel_mag_settings.acceleration_full_scale_selection << 3);
        try!(accel_mag.smbus_write_byte_data(0x21, accel_ctrl_reg_2));

        let mut accel_ctrl_reg_5: u8 = try!(accel_mag.smbus_read_byte_data(0x24)) & 0b10000011;
        accel_ctrl_reg_5 |= (accel_mag_settings.magnetic_resolution as u8) | (accel_mag_settings.magnetic_data_rate_selection << 2);
        try!(accel_mag.smbus_write_byte_data(0x24, accel_ctrl_reg_5));

        let mut accel_ctrl_reg_6: u8 = 0_u8;
        accel_ctrl_reg_6 |= accel_mag_settings.magnetic_full_scale_selection << 5;
        try!(accel_mag.smbus_write_byte_data(0x25, accel_ctrl_reg_6));

        let mut accel_ctrl_reg_7: u8 = 0_u8;
        accel_ctrl_reg_7 |= (accel_mag_settings.acceleration_high_pass_filter_mode as u8) | (accel_mag_settings.magnetic_sensor_mode as u8);
        if accel_mag_settings.acceleration_high_pass_filter {
            accel_ctrl_reg_7 |= 0b00100000;
        }
        if accel_mag_settings.magnetic_data_low_power_mode {
            accel_ctrl_reg_7 |= 0b00000100;
        }
        try!(accel_mag.smbus_write_byte_data(0x26, accel_ctrl_reg_7));

        // Set gyroscope settings
        let mut gyro_ctrl_reg_1: u8 = 0_u8 | gyro_settings.DR << 6 | gyro_settings.BW << 4;
        match gyro_settings.power_mode {
            LSM9DS0PowerMode::PowerDown => {},
            LSM9DS0PowerMode::Sleep => {
                gyro_ctrl_reg_1 |= (LSM9DS0PowerMode::Normal as u8);
            },
            LSM9DS0PowerMode::Normal => {
                gyro_ctrl_reg_1 |= (LSM9DS0PowerMode::Normal as u8);
                if gyro_settings.zen { gyro_ctrl_reg_1 |= 0b00000100; }
                if gyro_settings.yen { gyro_ctrl_reg_1 |= 0b00000010; }
                if gyro_settings.xen { gyro_ctrl_reg_1 |= 0b00000001; }
            }
        }
        try!(gyro.smbus_write_byte_data(0x20, gyro_ctrl_reg_1));

        let mut gyro_ctrl_reg_2: u8 = 0_u8 | (gyro_settings.filter_mode as u8) | gyro_settings.high_pass_filter_cutoff;
        gyro_ctrl_reg_2 &= 0b00111111;

        Ok(LSM9DS0 {
            accelerometer_magnetometer: accel_mag,
            gyroscope: gyro
        })
    }

    fn read_gyroscope_raw(&mut self) -> i32{
        0
    }

    fn read_magnetometer_raw(&mut self) -> i32 {
        0
    }

    fn read_accelerometer_raw(&mut self) -> i32 {
        0
    }


}

impl<T> Accelerometer for LSM9DS0<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    /// Returns reading in gs
    fn acceleration_reading(&mut self) -> Result<Vec3, T::Error> {
        let data = try!(self.gyroscope.smbus_read_byte_data(0x20));
        Ok(Vec3::zeros())
    }
}

impl<T> Magnetometer for LSM9DS0<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    /// Returns reading in gauss
    fn magnetic_reading(&mut self) -> Result<Vec3, T::Error> {
        Ok(Vec3::zeros())

    }
}

impl<T> Gyroscope for LSM9DS0<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;
    /// Returns reading in dps
    fn angular_rate_reading(&mut self) -> Result<Vec3, T::Error> {
        Ok(Vec3::zeros())

    }

}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
    }
}
