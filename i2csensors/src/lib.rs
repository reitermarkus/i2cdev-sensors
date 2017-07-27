// Copyright 2015, Paul Osborne <osbpau@gmail.com>
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/license/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option.  This file may not be copied, modified, or distributed
// except according to those terms.

use std::error::Error;

#[derive(Debug)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32
}

impl Vec3 {
    pub fn zeros() -> Vec3 {
        Vec3 { x: 0.0, y: 0.0, z: 0.0 }
    }
}

/// Trait for sensors that provide access to accelerometer readings (3-axis)
pub trait Accelerometer {
    type Error: Error;

    /// Returns 'Ok(accel)' if available, otherwise returns 'Err(Self::Error)'
    fn acceleration_reading(&mut self) -> Result<Vec3, Self::Error>;
}

pub trait Gyroscope {
    type Error: Error;

    /// Returns 'Ok(ang_rate)' if available, otherwise returns 'Err(Self::Error)'
    fn angular_rate_reading(&mut self) -> Result<Vec3, Self::Error>;
}

pub trait Magnetometer {
    type Error: Error;

    /// Returns 'Ok(mag)' if available, otherwise returns 'Err(Self::Error)'
    fn magnetic_reading(&mut self) -> Result<Vec3, Self::Error>;
}

/// Trait for sensors that provide access to temperature readings
pub trait Thermometer {
    type Error: Error;

    /// Get a temperature from the sensor in degrees celsius
    ///
    /// Returns `Ok(temperature)` if available, otherwise returns `Err(Self::Error)`
    fn temperature_celsius(&mut self) -> Result<f32, Self::Error>;
}

/// Trait for sensors that provide access to pressure readings
pub trait Barometer {
    type Error: Error;

    /// Get a pressure reading from the sensor in kPa
    ///
    /// Returns `Ok(temperature)` if avialable, otherwise returns `Err(Self::Error)`
    fn pressure_kpa(&mut self) -> Result<f32, Self::Error>;
}

/// Trait for sensors that provide access to altitude readings
pub trait Altimeter {
    type Error: Error;

    /// Get an altitude reading from the sensor in meters, relative to the pressure in kPa at
    /// sea level
    ///
    /// Returns `Ok(altitude)` if available, otherwise returns `Err(Self::Error)`
    fn altitude_meters(&mut self, sea_level_kpa: f32) -> Result<f32, Self::Error>;
}

impl<T> Altimeter for T
    where T: Barometer
{
    type Error = <Self as Barometer>::Error;

    fn altitude_meters(&mut self, sea_level_kpa: f32) -> Result<f32, Self::Error> {
        let pressure = try!(self.pressure_kpa()) * 1000.;
        let sea_level_pa = sea_level_kpa * 1000.;

        let altitude = 44330. * (1. - (pressure / sea_level_pa).powf(0.1903));
        Ok(altitude)
    }
}



