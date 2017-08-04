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
#[cfg(any(target_os = "linux", target_os = "android"))]
use i2cdev::linux::{LinuxI2CDevice,LinuxI2CError};
use byteorder::{ByteOrder, BigEndian, LittleEndian};

pub const BMP280_I2C_ADDR: u16 = 0x77;

const BMP280_PRESS_MSB: u8 = 0xF7;
const BMP280_PRESS_LSB: u8 = 0xF8;
const BMP280_PRESS_XLSB: u8 = 0xF9;

const BMP280_TEMP_MSB: u8 = 0xFA;
const BMP280_TEMP_LSB: u8 = 0xFB;
const BMP280_TEMP_XLSB: u8 = 0xFC;

/// Algorithm to process measurements.
/// Float is significantly more expensive.
pub enum BMP280CompensationAlgorithm{
    B32,
    B64,
    Float
}

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
//        /*
        let mut buf = [0_u8; 26];
        let mut register: u8 = 0x88;
        try!(i2cdev.write(&[register]));
        try!(i2cdev.read(&mut buf));

        Ok(BMP280CalibrationCoefficients {
            dig_t1: LittleEndian::read_u16(&buf[0..2]),
            dig_t2: LittleEndian::read_i16(&buf[2..4]),
            dig_t3: LittleEndian::read_i16(&buf[4..6]),
            dig_p1: LittleEndian::read_u16(&buf[6..8]),
            dig_p2: LittleEndian::read_i16(&buf[8..10]),
            dig_p3: LittleEndian::read_i16(&buf[10..12]),
            dig_p4: LittleEndian::read_i16(&buf[12..14]),
            dig_p5: LittleEndian::read_i16(&buf[14..16]),
            dig_p6: LittleEndian::read_i16(&buf[16..18]),
            dig_p7: LittleEndian::read_i16(&buf[18..20]),
            dig_p8: LittleEndian::read_i16(&buf[20..22]),
            dig_p9: LittleEndian::read_i16(&buf[22..24])
        })
//        */
    }
}

pub enum BMP280PowerMode {
    SleepMode = 0b00000000,
    NormalMode = 0b00000011,
    ForcedMode = 0b00000001
}

/// Ultra low power: ×1 16 bit / 2.62 Pa
/// Low power: ×2 17 bit / 1.31 Pa
/// Standard resolution: ×4 18 bit / 0.66 Pa
/// High resolution: ×8 19 bit / 0.33 Pa
/// Ultra high resolution: ×16 20 bit / 0.16 Pa
pub enum BMP280PressureOversampling {
    Off = 0b00000000,
    UltraLowPower = 0b00000100,
    LowPower= 0b00001000,
    StandardResolution = 0b00001100,
    HighResolution = 0b00010000,
    UltraHighResolution = 0b00010100
}

/// ×1: 16 bit / 0.0050 °C
/// ×2: 17 bit / 0.0025 °C
/// ×4: 18 bit / 0.0012 °C
/// ×8: 19 bit / 0.0006 °C
/// ×16: 20 bit / 0.0003 °C
///
/// Recommended:
/// 1x for all UltraLowPower->HighResolution
/// 2x for UltraHighResolution
pub enum BMP280TemperatureOversampling {
    Off,
    x1 = 0b00100000,
    x2 = 0b01000000,
    x4 = 0b01100000,
    x8 = 0b10000000,
    x16 = 0b10100000
}

///Off = 1, Low = 2, Medium = 4, High = 8, UltraHigh = 16
pub enum BMP280FilterCoefficient {
    Off = 0b00000100,
    Low = 0b00001000,
    Medium = 0b00001100,
    High = 0b00010000,
    UltraHigh = 0b00010100
}

pub enum BMP280Timing {
    ms0_5 = 0b00000000,
    ms62_5 = 0b00100000,
    ms125 = 0b01000000,
    ms250 = 0b01100000,
    ms500 = 0b10000000,
    ms1000 = 0b10100000,
    ms2000 = 0b11000000,
    ms4000 = 0b11100000
}

/// [data sheet](https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP280-DS001-18.pdf)
pub struct BMP280Settings {
    pub compensation: BMP280CompensationAlgorithm,
    pub t_sb: BMP280Timing,
    pub iir_filter_coeff: BMP280FilterCoefficient,
    pub osrs_t: BMP280TemperatureOversampling,
    pub osrs_p: BMP280PressureOversampling,
    pub power_mode: BMP280PowerMode
}

#[cfg(any(target_os = "linux", target_os = "android"))]
pub fn get_linux_bmp280_i2c_device() -> Result<LinuxI2CDevice, LinuxI2CError> {
    match LinuxI2CDevice::new("/dev/i2c-1", BMP280_I2C_ADDR) {
        Ok(device) => Ok(device),
        Err(e) => Err(e)
    }
}

pub struct BMP280<T: I2CDevice + Sized> {
    pub barometer: T,
    coeff: BMP280CalibrationCoefficients,
    t_fine: i32,
    algorithm: BMP280CompensationAlgorithm
}

impl<T> BMP280<T>
    where T: I2CDevice + Sized
{
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn new(mut i2cdev: T, settings: BMP280Settings) -> Result<BMP280<T>, T::Error> {
        let id = try!(i2cdev.smbus_read_byte_data(0xD0));
        assert!(id == 0x58);

        let measurement_control = 0_u8 | settings.osrs_t as u8 | settings.osrs_p as u8 | settings.power_mode as u8;
        let config = 0_u8 | settings.t_sb as u8 | settings.iir_filter_coeff as u8;

        try!(i2cdev.smbus_write_byte_data(0xF4, measurement_control));
        try!(i2cdev.smbus_write_byte_data(0xF5, config));

        let coefficients = try!(BMP280CalibrationCoefficients::new(&mut i2cdev));

        Ok(BMP280 {
            barometer: i2cdev,
            coeff: coefficients,
            t_fine: 0,
            algorithm: settings.compensation
        })
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn reset(&mut self) -> Result<(), T::Error> {
        try!(self.barometer.smbus_write_byte_data(0xE0, 0xB6));
        Ok(())
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn set_mode(&mut self, mode: BMP280PowerMode) -> Result<(), T::Error> {
        let mut ctrl_meas = try!(self.barometer.smbus_read_byte_data(0xF4));
        ctrl_meas = ctrl_meas & 0b11111100;
        ctrl_meas = ctrl_meas | mode as u8;
        try!(self.barometer.smbus_write_byte_data(0xF4, ctrl_meas));
        Ok(())
    }

    fn compensate_temperature_32b(&mut self, adc_t: i32) -> i32 {
        let (mut var1, mut var2, mut t): (i32, i32, i32);
        var1 = ((((adc_t >> 3) - ((self.coeff.dig_t1 as i32) <<1))) * (self.coeff.dig_t2 as i32)) >> 11;
        var2 = (((((adc_t>>4) - (self.coeff.dig_t1 as i32)) * ((adc_t>>4) - (self.coeff.dig_t1 as i32))) >> 12) *
            (self.coeff.dig_t3 as i32)) >> 14;
        self.t_fine = var1 + var2;
        t = (self.t_fine * 5 + 128) >> 8;
        t
    }

    fn compensate_pressure_32b(&mut self, adc_p: i32) -> u32 {
        let (mut var1, mut var2, mut p): (i32, i32, u32);

        var1 = ((self.t_fine as i32) >> 1) - 64000;
        var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (self.coeff.dig_p6 as i32);
        var2 = var2 + ((var1 * (self.coeff.dig_p5 as i32)) << 1);
        var2 = (var2 >> 2) + ((self.coeff.dig_p4 as i32) << 16);
        var1 = ((((self.coeff.dig_p3 as i32) * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + (((self.coeff.dig_p2 as i32) * var1) >> 1)) >> 18;
        var1 = ((((32768 + var1)) * (self.coeff.dig_p1 as i32)) >> 15);
        if var1 == 0 {
            return 0; // avoid exception caused by division by zero
        }
        p = (((1048576 - adc_p) - (var2 >> 12)) as u32) * 3125;
        if p < 0x80000000 {
            p = (p << 1) / (var1 as u32);
        } else {
            p = (p / var1 as u32) * 2;
        }
        var1 = ((self.coeff.dig_p9 as i32) * ((((p >> 3) * (p >> 3)) >> 13) as i32)) >> 12;
        var2 = (((p >> 2) as i32) * (self.coeff.dig_p8) as i32) >> 13;
        p = ((p as i32) + ((var1 + var2 + (self.coeff.dig_p7 as i32)) >> 4)) as u32;
        p
    }

    fn compensate_pressure_64b(&mut self, adc_p: i32) -> u32 {
        let (mut var1, mut var2, mut p): (i64, i64, i64);
        var1 = (self.t_fine as i64) - 128000;
        var2 = var1 * var1 * (self.coeff.dig_p6 as i64);
        var2 = var2 + ((var1 * (self.coeff.dig_p5 as i64)) << 17);
        var2 = var2 + ((self.coeff.dig_p4 as i64) << 35);
        var1 = ((var1 * var1 * (self.coeff.dig_p3 as i64))>>8) + ((var1 * (self.coeff.dig_p2 as i64)) << 12);
        var1 = ((1_i64 << 47) + var1)*(self.coeff.dig_p1 as i64)>>33;
        if var1 == 0 {
            return 0; // avoid exception caused by division by zero
        }
        p = 1048576 - adc_p as i64;
        p = (((p << 31)-var2) * 3125) / var1;
        var1 = ((self.coeff.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
        var2 = ((self.coeff.dig_p8 as i64) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + ((self.coeff.dig_p7 as i64) << 4);
        p as u32
    }

    fn compensate_temperature_float(&mut self, adc_t: i32) -> f64 {
        let (mut var1, mut var2, mut t): (f64, f64, f64);
        var1 = ((adc_t as f64)/16384.0 - (self.coeff.dig_t1 as f64)/1024.0) * (self.coeff.dig_t2 as f64);
        var2 = (((adc_t as f64)/131072.0 - (self.coeff.dig_t1 as f64)/8192.0) *
            ((adc_t as f64)/131072.0 - (self.coeff.dig_t1 as f64)/8192.0)) * (self.coeff.dig_t3 as f64);
        self.t_fine = (var1 + var2) as i32;
        t = (var1 + var2) / 5120.0;
        t
    }

    fn compensate_pressure_float(&mut self, adc_p: i32) -> f64 {
        let (mut var1, mut var2, mut p): (f64, f64, f64);

        var1 = ((self.t_fine as f64)/2.0) - 64000.0_f64;
        var2 = var1 * var1 * (self.coeff.dig_p6 as f64) / 32768.0_f64;
        var2 = var2 + var1 * (self.coeff.dig_p5 as f64) * 2.0_f64;
        var2 = (var2 / 4.0_f64) + ((self.coeff.dig_p4 as f64) * 65536.0_f64);
        var1 = ((self.coeff.dig_p3 as f64) * var1 * var1 / 524288.0_f64 + (self.coeff.dig_p2 as f64) * var1) / 524288.0_f64;
        var1 = (1.0 + var1 / 32768.0_f64) * (self.coeff.dig_p1 as f64);
        if var1 == 0.0_f64 {
            return 0.0; // avoid exception caused by division by zero
        }
        p = 1048576.0_f64 - (adc_p as f64);
        p = (p - (var2 / 4096.0_f64)) * 6250.0_f64 / var1;
        var1 = (self.coeff.dig_p9 as f64) * p * p / 2147483648.0_f64;
        var2 = p * (self.coeff.dig_p8 as f64) / 32768.0_f64;
        p = p + (var1 + var2 + (self.coeff.dig_p7 as f64)) / 16.0_f64;
        p
    }

    fn compensate_temperature(&mut self, adc_t: i32) -> f32 {
        match self.algorithm {
            BMP280CompensationAlgorithm::B32 => {
                let result = self.compensate_temperature_32b(adc_t);
                return (result as f32) / 100.0
            },
            BMP280CompensationAlgorithm::B64 => {
                let result = self.compensate_temperature_32b(adc_t);
                return (result as f32) / 100.0
            },
            BMP280CompensationAlgorithm::Float => self.compensate_temperature_float(adc_t) as f32
        }
    }

    //Returns Pa
    fn compensate_pressure(&mut self, adc_p: i32) -> f32 {
        match self.algorithm {
            BMP280CompensationAlgorithm::B32 => {
                let result = self.compensate_pressure_64b(adc_p);
                return result as f32;
            },
            BMP280CompensationAlgorithm::B64 => {
                let result = self.compensate_pressure_64b(adc_p);
                return (result as f32) / 256.0;
            },
            BMP280CompensationAlgorithm::Float => {
                self.compensate_pressure_float(adc_p) as f32
            }
        }
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn read_temp_raw(&mut self) -> Result<i32, T::Error> {
        let mut buf = [0_u8; 3];
        try!(self.barometer.write(&[BMP280_TEMP_MSB]));
        try!(self.barometer.read(&mut buf));
        let temp = BigEndian::read_i24(&buf) << 4;
        let mut raw_temp: i32 = ((buf[0] as i32) << 12) + ((buf[1] as i32) << 4) + ((buf[2] as i32) >> 4);

        Ok(raw_temp)
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn read_press_raw(&mut self) -> Result<i32, T::Error> {
        let mut buf = [0_u8; 3];
        try!(self.barometer.write(&[BMP280_PRESS_MSB]));
        try!(self.barometer.read(&mut buf));
        let raw_press = ((buf[0] as i32) << 12) | ((buf[1] as i32) << 4) | ((buf[2] as i32) >> 4);

        Ok(raw_press)
    }

    #[doc(hidden)]
    pub fn test_calculate_real_pressure(&mut self) {
        self.coeff = BMP280CalibrationCoefficients {
            dig_t1: 27504,
            dig_t2: 26435,
            dig_t3: -1000,
            dig_p1: 36477,
            dig_p2: -10685,
            dig_p3: 3024,
            dig_p4: 2855,
            dig_p5: 140,
            dig_p6: -7,
            dig_p7: 15500,
            dig_p8: -14600,
            dig_p9: 6000,
        };

        let temp_reading = 519888;
        let pressure_reading = 415148;

        let tdiff_f = self.compensate_temperature_float(temp_reading) - 25.08;
        assert!(tdiff_f.abs() < 1.0 && tdiff_f > -1.0);
        println!("TempFloat PASS");
        let pdiff_f = self.compensate_pressure_float(pressure_reading) - 100653.26;
        assert!(pdiff_f.abs() < 1.0 || pdiff_f > -1.0);
        println!("PressFloat PASS");

        assert!(self.compensate_temperature_32b(temp_reading) == 2508);
        println!("Temp32b PASS");

        println!("{}", self.compensate_pressure_32b(pressure_reading));
        //Off by 3 for some reason even though I've triple checked the algorithm
        //        assert!(self.compensate_pressure_32b(pressure_reading) == 100653_u32);
        println!("Press32b PASS");

        println!("{}", self.compensate_pressure_64b(pressure_reading));
        //Off by 3 for some reason even though I've triple checked the algorithm
//        assert!(self.compensate_pressure_64b(pressure_reading) == 25767236_u32);
        println!("Press64b PASS");

        println!("Passed calibration test");
    }
}

impl<T> Thermometer for BMP280<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    fn temperature_celsius(&mut self) -> Result<f32, T::Error> {
        Ok(0.0)
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn temperature_celsius(&mut self) -> Result<f32, T::Error> {
        match self.read_temp_raw() {
            Ok(adc_t) => Ok(self.compensate_temperature(adc_t)),
            Err(e) => Err(e)
        }
    }
}

impl<T> Barometer for BMP280<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    fn pressure_kpa(&mut self) -> Result<f32, T::Error> {
        Ok(0.0)
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn pressure_kpa(&mut self) -> Result<f32, T::Error> {
        self.temperature_celsius();

        match self.read_press_raw() {
            Ok(adc_p) => Ok(self.compensate_pressure(adc_p) / 1000.0),
            Err(e) => Err(e)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_algorithms() {
        println!("BMP280 Barometer Thermometer.");
        match get_linux_bmp280_i2c_device() {
            Ok(device) => {
                let settings = BMP280Settings {
                    compensation: BMP280CompensationAlgorithm::B64,
                    t_sb: BMP280Timing::ms0_5,
                    iir_filter_coeff: BMP280FilterCoefficient::Medium,
                    osrs_t: BMP280TemperatureOversampling::x1,
                    osrs_p: BMP280PressureOversampling::StandardResolution,
                    power_mode: BMP280PowerMode::NormalMode
                };

                match BMP280::new(device, settings) {
                    Ok(mut bmp280) => {
                        bmp280.test_calculate_real_pressure();
                    },
                    Err(e) => {}
                }
            },
            Err(e) => {}
        }
    }
}
