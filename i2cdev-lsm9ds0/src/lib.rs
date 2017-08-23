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
#[cfg(any(target_os = "linux", target_os = "android"))]
use i2cdev::linux::{LinuxI2CDevice,LinuxI2CError};
use byteorder::{ByteOrder, BigEndian, LittleEndian};

// Addresses
pub const LSM9DS0_I2C_ADDR_GYRO: u16 = 0x6A;
pub const LSM9DS0_I2C_ADDR_ACCEL_MAG: u16 = 0x1E;

const LSM9DS0_CTRL_REG1: u8 = 0x20;
const LSM9DS0_CTRL_REG2: u8 = 0x21;
const LSM9DS0_CTRL_REG3: u8 = 0x22;
const LSM9DS0_CTRL_REG4: u8 = 0x23;
const LSM9DS0_CTRL_REG5: u8 = 0x24;
const LSM9DS0_CTRL_REG6: u8 = 0x25;
const LSM9DS0_CTRL_REG7: u8 = 0x26;

const LSM9DS0_INCREMENT_BIT: u8 = 0x80;

const LSM9DS0_OUT_GYRO: u8 = 0x28;
const LSM9DS0_OUT_MAG: u8 = 0x08;
const LSM9DS0_OUT_ACC: u8 = 0x28;

const LSM9DS0_WHO_AM_I_GYRO: u8 = 0x0F;
const LSM9DS0_ID_GYRO: u8 = 0b11010100;
const LSM9DS0_WHO_AM_I_ACCEL: u8 = 0x0F;
const LSM9DS0_ID_ACCEL: u8 = 0b01001001;

// Settings

// Gyroscope
#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0PowerMode {
    PowerDown = 0b00000000,
    Sleep,
    Normal = 0b00001000
}

#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0GyroscopeDataRate {
    Hz95 = 0b00,
    Hz190 = 0b01,
    Hz380 = 0b10,
    Hz760 = 0b11
}

/// Reference table 21 on data sheet
/// I believe this is the cutoff of the low pass filter
#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0GyroscopeBandwidth {
    BW1 = 0b00,
    BW2 = 0b01,
    BW3 = 0b10,
    BW4 = 0b11
}

#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0GyroscopeFS {
    dps250 = 0b00,
    dps500 = 0b01,
    dps2000 = 0b10
}

#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0GyroscopeHighPassFilterMode {
    NormalModeHPRESETFILTER = 0b00,
    ReferenceSignalForFiltering = 0b01,
    NormalMode = 0b10,
    AutoresetOnInterruptEvent = 0b11
}

#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0HighPassFilterCutOffConfig {
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
pub struct LSM9DS0GyroscopeSettings {
    /// Data measurement rate
    pub DR: LSM9DS0GyroscopeDataRate,
    /// Low pass filter cutoff
    pub BW: LSM9DS0GyroscopeBandwidth,
    /// Sleep will automatically disable '''xen''', '''yen''', '''zen'''
    pub power_mode: LSM9DS0PowerMode,
    /// Enable z axis readings
    pub zen: bool,
    /// Enable y axis readings
    pub yen: bool,
    /// Enable x axis readings
    pub xen: bool,
    /// Range of measurements. Lower range means more precision.
    pub sensitivity: LSM9DS0GyroscopeFS,
    /// Set to false if you do not want to update the buffer unless it has been read
    pub continuous_update: bool,
    pub high_pass_filter_enabled: bool,
    pub high_pass_filter_mode: Option<LSM9DS0GyroscopeHighPassFilterMode>,
    pub high_pass_filter_configuration: Option<LSM9DS0HighPassFilterCutOffConfig>
}

// Accelerometer/Magnetometer
#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0AccelerometerUpdateRate {
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
pub enum LSM9DS0AccelerometerFilterBandwidth {
    Hz773 = 0b00,
    Hz194 = 0b01,
    Hz362 = 0b10,
    Hz50 = 0b11
}

#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0AccelerometerFS {
    g2 = 0b000,
    g4 = 0b001,
    g6 = 0b010,
    g8 = 0b011,
    g16 = 0b100
}

#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0MagnetometerMode {
    ContinuousConversion = 0b00,
    SingleConversion = 0b01,
    PowerDown = 0b10
}

#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0MagnetometerResolution {
    Low = 0b00,
    MediumLow = 0b01,
    MediumHigh = 0b10,
    High = 0b11
}

#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0MagnetometerUpdateRate {
    Hz3_125 = 0b000,
    Hz6_25 = 0b001,
    Hz12_5 = 0b010,
    Hz25 = 0b011,
    Hz50 = 0b100,
    Hz100 = 0b101
}

#[derive(Debug, Copy, Clone)]
pub enum LSM9DS0MagnetometerFS {
    gauss2 = 0b00,
    gauss4 = 0b01,
    gauss8 = 0b10,
    gauss12 = 0b11
}

/// Use the [data sheet](http://www.st.com/content/ccc/resource/technical/document/datasheet/1c/9e/71/05/4e/b7/4d/d1/DM00057547.pdf/files/DM00057547.pdf/jcr:content/translations/en.DM00057547.pdf) to read in depth about settings
#[derive(Debug, Copy, Clone)]
pub struct LSM9DS0AccelerometerMagnetometerSettings {
    /// Continuously update output registers or wait until read
    pub continuous_update: bool,
    //Accelerometer
    /// Frequency that accelerometer measurements are made
    pub accelerometer_data_rate: LSM9DS0AccelerometerUpdateRate,
    pub accelerometer_anti_alias_filter_bandwidth: LSM9DS0AccelerometerFilterBandwidth,
    /// Enable accelerometer z axis
    pub azen: bool,
    /// Enable accelerometer y axis
    pub ayen: bool,
    /// Enable accelerometer x axis
    pub axen: bool,
    /// The maximum/minimum (+-) reading of acceleration (Full range). Smaller ranges have more precision.
    pub accelerometer_sensitivity: LSM9DS0AccelerometerFS,
    //Magnetometer
    pub magnetometer_resolution: LSM9DS0MagnetometerResolution,
    /// Frequency that magnetometer measurements are made
    pub magnetometer_data_rate: LSM9DS0MagnetometerUpdateRate,
    pub magnetometer_low_power_mode: bool,
    pub magnetometer_mode: LSM9DS0MagnetometerMode,
    /// The maximum/minimum (+-) reading of magnetism (Full range). Smaller ranges have more precision.
    pub magnetometer_sensitivity: LSM9DS0MagnetometerFS
}

// Public function

/// Returns (gyroscope, accelerometer/magnetometer) at their default I2C addresses.
#[cfg(any(target_os = "linux", target_os = "android"))]
pub fn get_default_lsm9ds0_linux_i2c_devices() -> Result<(LinuxI2CDevice,LinuxI2CDevice),LinuxI2CError> {
    let gyro = try!(LinuxI2CDevice::new("/dev/i2c-1", LSM9DS0_I2C_ADDR_GYRO));
    let accel_mag = try!(LinuxI2CDevice::new("/dev/i2c-1", LSM9DS0_I2C_ADDR_ACCEL_MAG));
    Ok((gyro,accel_mag))
}

#[derive(Copy,Clone)]
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
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn new(mut accel_mag: T, mut gyro: T,
               mut gyro_settings: LSM9DS0GyroscopeSettings,
               mut accel_mag_settings: LSM9DS0AccelerometerMagnetometerSettings) -> Result<LSM9DS0<T>, T::Error>
    {
        // Check that the device is actually an LSM9DS0 chip.
        assert!(try!(gyro.smbus_read_byte_data(LSM9DS0_WHO_AM_I_GYRO)) == LSM9DS0_ID_GYRO, "LSM9DS0 gyroscope ID didn't match for device at given I2C address.");
        assert!(try!(accel_mag.smbus_read_byte_data(LSM9DS0_WHO_AM_I_ACCEL)) == LSM9DS0_ID_ACCEL, "LSM9DS0 accelerometer/magnetometer ID didn't match for device at given I2C address.");

        // Try to set the control registers.

        // Gyroscope
        let mut ctrl_reg1: u8 = 0_u8 | ((gyro_settings.DR as u8) << 6) | ((gyro_settings.BW as u8) << 4);
        match gyro_settings.power_mode {
            LSM9DS0PowerMode::PowerDown => {
                ctrl_reg1 |= LSM9DS0PowerMode::PowerDown as u8;
            },
            LSM9DS0PowerMode::Sleep => {
                ctrl_reg1 = (ctrl_reg1 | LSM9DS0PowerMode::Normal as u8) & 0b11111000;
            },
            LSM9DS0PowerMode::Normal => {
                ctrl_reg1 |= LSM9DS0PowerMode::Normal as u8;
                if gyro_settings.xen { ctrl_reg1 |= 0b001 };
                if gyro_settings.yen { ctrl_reg1 |= 0b010 };
                if gyro_settings.zen { ctrl_reg1 |= 0b100 };
            }
        }
        try!(gyro.smbus_write_byte_data(LSM9DS0_CTRL_REG1, ctrl_reg1));

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
        try!(gyro.smbus_write_byte_data(LSM9DS0_CTRL_REG2, ctrl_reg2));

        let mut ctrl_reg4: u8 = 0_u8 | ((gyro_settings.sensitivity as u8) << 4);
        if !gyro_settings.continuous_update {
            ctrl_reg4 |= 0b10000000;
        }
        try!(gyro.smbus_write_byte_data(LSM9DS0_CTRL_REG4, ctrl_reg4));

        let mut ctrl_reg5: u8 = 0_u8;
        if gyro_settings.high_pass_filter_enabled {
            ctrl_reg5 = 0b00010000;
        }
        try!(gyro.smbus_write_byte_data(LSM9DS0_CTRL_REG5, ctrl_reg5));

        // Calculate gain
        let mut g_gain: f32;

        match gyro_settings.sensitivity {
            LSM9DS0GyroscopeFS::dps250 => {
                g_gain = 8.75;
            },
            LSM9DS0GyroscopeFS::dps500 => {
                g_gain = 17.50;
            },
            LSM9DS0GyroscopeFS::dps2000 => {
                g_gain = 70.0;
            }
        }

        // Accelerometer/Magnetometer
        let mut ctrl_reg1: u8 = 0_u8 | ((accel_mag_settings.accelerometer_data_rate as u8) << 4);
        if !accel_mag_settings.continuous_update {
            ctrl_reg1 |= 0b00001000;
        }
        if accel_mag_settings.axen { ctrl_reg1 |= 0b001 };
        if accel_mag_settings.ayen { ctrl_reg1 |= 0b010 };
        if accel_mag_settings.azen { ctrl_reg1 |= 0b100 };
        try!(accel_mag.smbus_write_byte_data(LSM9DS0_CTRL_REG1, ctrl_reg1));

        let mut ctrl_reg2: u8 = 0_u8 | ((accel_mag_settings.accelerometer_sensitivity as u8) << 3) | ((accel_mag_settings.accelerometer_anti_alias_filter_bandwidth as u8) << 6);
        try!(accel_mag.smbus_write_byte_data(LSM9DS0_CTRL_REG2, ctrl_reg2));

        let mut ctrl_reg5: u8 = 0_u8 | ((accel_mag_settings.magnetometer_resolution as u8) << 5) | ((accel_mag_settings.magnetometer_data_rate as u8) << 2); //24
        try!(accel_mag.smbus_write_byte_data(LSM9DS0_CTRL_REG5, ctrl_reg5));

        let mut ctrl_reg6: u8 = 0_u8 | ((accel_mag_settings.magnetometer_sensitivity as u8) << 5);
        try!(accel_mag.smbus_write_byte_data(LSM9DS0_CTRL_REG6, ctrl_reg6));

        let mut ctrl_reg7: u8 = 0_u8 | (accel_mag_settings.magnetometer_mode as u8);
        if accel_mag_settings.magnetometer_low_power_mode {
            ctrl_reg7 |= 0b00000100;
        }
        try!(accel_mag.smbus_write_byte_data(LSM9DS0_CTRL_REG7, ctrl_reg7));

        // Calculate gains
        let mut a_gain: f32;
        let mut m_gain: f32;

        match accel_mag_settings.magnetometer_sensitivity {
            LSM9DS0MagnetometerFS::gauss2 => {
                m_gain = 0.08;
            },
            LSM9DS0MagnetometerFS::gauss4 => {
                m_gain = 0.160;
            },
            LSM9DS0MagnetometerFS::gauss8 => {
                m_gain = 0.320;
            },
            LSM9DS0MagnetometerFS::gauss12 => {
                m_gain = 0.479;
            }
        }

        match accel_mag_settings.accelerometer_sensitivity {
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
            }
        }

        Ok(LSM9DS0 {
            accelerometer_magnetometer: accel_mag,
            gyroscope: gyro,
            g_gain: g_gain / 1000.0,
            a_gain: a_gain / 1000.0,
            m_gain: m_gain / 1000.0
        })

    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn read_gyroscope_raw(&mut self) -> Result<(i16, i16, i16), T::Error>{
        let mut buf = [0_u8; 6];
        try!(self.gyroscope.write(&[LSM9DS0_INCREMENT_BIT | LSM9DS0_OUT_GYRO]));
        try!(self.gyroscope.read(&mut buf));
        let x_raw = LittleEndian::read_i16(&buf[0..2]);
        let y_raw = LittleEndian::read_i16(&buf[2..4]);
        let z_raw = LittleEndian::read_i16(&buf[4..6]);
        Ok((x_raw, y_raw, z_raw))
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn read_magnetometer_raw(&mut self) -> Result<(i16,i16,i16), T::Error> {
        let mut buf = [0_u8; 6];
        try!(self.accelerometer_magnetometer.write(&[LSM9DS0_INCREMENT_BIT | LSM9DS0_OUT_MAG]));
        try!(self.accelerometer_magnetometer.read(&mut buf));
        let x_raw = LittleEndian::read_i16(&buf[0..2]);
        let y_raw = LittleEndian::read_i16(&buf[2..4]);
        let z_raw = LittleEndian::read_i16(&buf[4..6]);
        Ok((x_raw, y_raw, z_raw))
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn read_accelerometer_raw(&mut self) -> Result<(i16, i16, i16), T::Error> {
        let mut buf: [u8; 6] = [0_u8; 6];
        try!(self.accelerometer_magnetometer.write(&[LSM9DS0_INCREMENT_BIT | LSM9DS0_OUT_ACC]));
        try!(self.accelerometer_magnetometer.read(&mut buf));
        let x_raw = LittleEndian::read_i16(&buf[0..2]);
        let y_raw = LittleEndian::read_i16(&buf[2..4]);
        let z_raw = LittleEndian::read_i16(&buf[4..6]);
        Ok((x_raw, y_raw, z_raw))
    }
}

impl<T> Accelerometer for LSM9DS0<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;
    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    fn acceleration_reading(&mut self) -> Result<Vec3, T::Error> {
        Ok(Vec3::zeros())
    }

    /// Returns reading in gs
    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn acceleration_reading(&mut self) -> Result<Vec3, T::Error> {
        let (x_raw, y_raw, z_raw) = try!(self.read_accelerometer_raw());
        let acceleration = Vec3 {
            x: (x_raw as f32) * self.a_gain,
            y: (y_raw as f32) * self.a_gain,
            z: (z_raw as f32) * self.a_gain
        };

        Ok(acceleration)
    }
}

impl<T> Magnetometer for LSM9DS0<T>
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
        let (x_raw, y_raw, z_raw) = try!(self.read_magnetometer_raw());
        let magnetic_reading = Vec3 {
            x: (x_raw as f32) * self.m_gain,
            y: (y_raw as f32) * self.m_gain,
            z: (z_raw as f32) * self.m_gain
        };

        Ok(magnetic_reading)
    }
}

impl<T> Gyroscope for LSM9DS0<T>
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

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
    }
}
