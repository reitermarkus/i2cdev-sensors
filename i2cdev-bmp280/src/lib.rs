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

use i2csensors::{Barometer, Thermometer};
use std::thread;
use std::time::Duration;
use std::error::Error;
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;
use byteorder::{ByteOrder, BigEndian};

pub const BMP280_I2C_ADDR: u16 = 0x77;

const BMP280_PRESS_MSB: u16 = 0xF7;
const BMP280_PRESS_LSB: u16 = 0xF8;
const BMP280_PRESS_XLSB: u16 = 0xF9;

const BMP280_TEMP_MSB: u16 = 0xFA;
const BMP280_TEMP_LSB: u16 = 0xFB;
const BMP280_TEMP_XLSB: u16 = 0xFC;

struct BMP280CalibrationCoefficients {
    pub dig_t1: u16,
    pub dig_t2: i16,
    pub dig_t3: i16,
    pub dig_p1: u16,
    pub dig_p2: i16,
    pub dig_p3: i16,
    pub dig_p4: i16,
    pub dig_p5: i16,
    pub dig_p6: i16,
    pub dig_p7: i16,
    pub dig_p8: i16,
    pub dig_p9: i16,
}

impl BMP280CalibrationCoefficients {
    pub fn new<E: Error>(i2cdev: &mut I2CDevice<Error = E>) -> Result<BMP280CalibrationCoefficients, E> {
        let mut buf: [u8; 24] = [0; 24];
        let mut register: u16 = 0x88;
        for i in 24 {
            match i2cdev.smbus_read_byte_data(register) {
                Ok(byte) => {
                    &buf[i] = byte;
                },
                Err(e) => {
                    println!("Error reading BMP280 calibration coefficient.");
                    return Err(e);
                }
            }
            register += 1;
        }

        Ok(BMP280CalibrationCoefficients {
            dig_t1: BigEndian::read_u16(&buf[0..2]),
            dig_t2: BigEndian::read_i16(&buf[2..4]),
            dig_t3: BigEndian::read_i16(&buf[4..6]),
            dig_p1: BigEndian::read_u16(&buf[6..8]),
            dig_p2: BigEndian::read_i16(&buf[8..10]),
            dig_p3: BigEndian::read_i16(&buf[10..12]),
            dig_p4: BigEndian::read_i16(&buf[12..14]),
            dig_p5: BigEndian::read_i16(&buf[14..16]),
            dig_p6: BigEndian::read_i16(&buf[16..18]),
            dig_p7: BigEndian::read_i16(&buf[18..20]),
            dig_p8: BigEndian::read_i16(&buf[20..22]),
            dig_p9: BigEndian::read_i16(&buf[22..24])
        })
    }
}

pub enum PowerMode {
    SleepMode = 0b00,
    NormalMode = 0b11,
    ForcedMode = 0b01
}

pub enum PressureOversampling {
    x1 = 0b001,
    x2 = 0b010,
    x4 = 0b011,
    x8 = 0b100,
    x16 = 0b101
}

pub enum TempuratureOversampling {
    x1 = 0b001,
    x2 = 0b010,
    x4 = 0b011,
    x8 = 0b100,
    x16 = 0b101
}

pub enum SamplingSettings {
    TemperatureMeasurement,
    UltraLowPower,
    LowPower,
    StandardResolution,
    HighResolution,
    UltraHighResolution
}

pub struct BMP280Settings {

}

pub struct BMP280<T: I2CDevice + Sized> {
    pub barometer: T,
    pub coeff: BMP280CalibrationCoefficients,
    t_fine: i32
}

impl<T> BMP280<T>
    where T: I2CDevice + Sized
{
    fn new(i2cdev: T) -> BMP280<T> {
        let new_bmp280: BMP280<T> = BMP280 {
            barometer: i2cdev,
            coeff: BMP280CalibrationCoefficients::new(&i2cdev),
            t_fine: 0.0
        };
    }

    fn get_linux_i2c_device() -> Result<LinuxI2CDevice, std::string::String> {
        match LinuxI2CDevice::new("/dev/i2c-1", BMP280_I2C_ADDR) {
            Ok(device) => device,
            Err(e) => Err(e)
        }
    }

    fn compensate_temperature(&mut self, adc_t: i32) -> f64 {
        let var1: f64;
        let var2: f64;
        let t: f64;
        var1 = ((adc_t as f64)/16384.0 - (self.coeff.dig_t1 as f64)/1024.0) * (self.coeff.dig_t2 as f64);
        var2 = (((adc_t as f64)/131072.0 - (self.coeff.dig_t1 as f64)/8192.0) *
            ((adc_t as f64)/131072.0 - (self.coeff.dig_t1 as f64)/8192.0)) * (self.coeff.dig_t3 as f64);
        self.t_fine = (var1 + var2) as i32;
        t = (var1 + var2) / 5120.0;
        t
    }

    fn compensate_pressure(&mut self, adc_p: i32) -> f64 {
        let var1 : i64;
        let var2: i64;
        let p: i64;

        var1 = ((self.t_fine as f64)/2.0) - 64000.0;
        var2 = var1 * var1 * (self.coeff.dig_p6 as f64) / 32768.0;
        var2 = var2 + var1 * (self.coeff.dig_p5 as f64) * 2.0;
        var2 = (var2/4.0)+((self.coeff.dig_p4 as f64) * 65536.0);
        var1 = ((self.coeff.dig_p3 as f64) * var1 * var1 / 524288.0 + (self.coeff.dig_p2 as f64) * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0)*(self.coeff.dig_p1 as f64);
        if var1 == 0.0 {
            return 0; // avoid exception caused by division by zero
        }
        p = 1048576.0 - adc_p as f64;
        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = (self.coeff.dig_p9 as f64) * p * p / 2147483648.0;
        var2 = p * (self.coeff.dig_p8 as f64) / 32768.0;
        p = p + (var1 + var2 + (self.coeff.dig_p7 as f64)) / 16.0;
        p
    }

    fn read_temp_raw(&mut self) -> Result<i32,std::string::String> {
        match self.barometer.smbus_read_block_data() {
            Ok(bytes) => BigEndian::read_i32(bytes.as_slice()),
            Err(e) => Err(e)
        }
    }

    fn read_press_raw(&mut self) -> Result<i32,std::string::String> {
        match self.barometer.smbus_read_block_data() {
            Ok(bytes) => BigEndian::read_i32(bytes.as_slice()),
            Err(e) => Err(e)
        }
    }
}

impl<T> Thermometer for BMP280<T>
    where T: I2CDevice + Sized
{
    type Error = Error<std::string::String>;

    fn temperature_celsius(&mut self) -> Result<f32, Error> {
        let adc_t = self.read_temp_raw();
        self.compensate_temperature(adc_t)
    }
}

impl<T> Barometer for BMP280<T>
    where T: I2CDevice + Sized
{
    type Error = Error<std::string::String>;

    fn pressure_kpa(&mut self) -> Result<f32, Error> {
        let adc_p = self.read_press_raw();
        self.compensate_pressure(adc_p)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
    }
}
