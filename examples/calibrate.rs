use bno055::{BNO055OperationMode, Bno055}; //sensor
                                           //use linux_embedded_hal::{Delay, I2cdev};
use anyhow::Result;
use esp_idf_hal::delay::Ets; // équivalent de Delay
use esp_idf_hal::i2c::*; // I2C peripheral
use esp_idf_hal::peripherals::Peripherals;
use mint::{EulerAngles, Quaternion}; //mathg tool to calculate angle
//use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs; // (utile pour Delay aussi)
//use esp_idf_hal::units::FromValueType;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::units::Hertz;

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let config = I2cConfig::new().baudrate(Hertz::from(100_000)); //max 400kHz
                                                                  //let config = I2cConfig::new().baudrate(100_i32.kHz().into());
    let sda = peripherals.pins.gpio6;
    let scl = peripherals.pins.gpio7;
    let mut i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config)?;

    //test
    println!(" Scanning I2C bus..."); //send an empty trame to every possible i2c adress
    for addr in 0b0..0b00101001 {
        if i2c.write(addr, &[], 100).is_ok() {
            //if write dont have error, a device exist at this adress
            println!("Device found at 0x{:02X}", addr); //print if found a device
        }
    }

    let mut delay = Ets {};

    let mut imu = Bno055::new(i2c).with_alternative_address();

    FreeRtos::delay_ms(700); //wait for the sensor to be ready

    println!("Try to initialize the sensor");
    match imu.init(&mut delay) {
        Ok(_) => println!("Capteur initialisé "),
        Err(e) => {
            println!("Erreur init: {:?}", e);
            let chip_id = imu.id().unwrap_or(0x00);
            println!("Chip ID reçu = 0x{:02X}", chip_id);
        }
    };

    // /*
    // let dev = I2cdev::new("/dev/i2c-0").unwrap();
    // let mut delay = Delay {};
    // let mut imu = Bno055::new(dev).with_alternative_address();
    // */

     imu.init(&mut delay)
    //     .expect("An error occurred while building the IMU");

    imu.set_mode(BNO055OperationMode::NDOF, &mut delay)
        .expect("An error occurred while setting the IMU mode");

    let mut status = imu.get_calibration_status().unwrap();
    // println!("The IMU's calibration status is: {:?}", status);

    // // Wait for device to auto-calibrate.
    // // Please perform steps necessary for auto-calibration to kick in.
    // // Required steps are described in Datasheet section 3.11
    // // Page 51, https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf (As of 2021-07-02)
    // println!("- About to begin BNO055 IMU calibration...");
    / while !imu.is_fully_calibrated().unwrap() {
    //     status = imu.get_calibration_status().unwrap();
    //     std::thread::sleep(std::time::Duration::from_millis(1000));
    //     println!("Calibration status: {:?}", status);
    // }

    // let calib = imu.calibration_profile(&mut delay).unwrap();

    // imu.set_calibration_profile(calib, &mut delay).unwrap();
    // println!("       - Calibration complete!");

    // // These are sensor fusion reading using the mint crate that the state will be read into
    // let mut euler_angles: EulerAngles<f32, ()>; // = EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]);
    // let mut quaternion: Quaternion<f32>; // = Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]);

    // loop {
    //     // Quaternion; due to a bug in the BNO055, this is recommended over Euler Angles
         match imu.quaternion() {
    //         Ok(val) => {
    //             quaternion = val;
    //             println!("IMU Quaternion: {:?}", quaternion);
    //             std::thread::sleep(std::time::Duration::from_millis(500));
    //         }
    //         Err(e) => {
    //             eprintln!("{:?}", e);
    //         }
    //     }

    //     // Euler angles, directly read
     //   match imu.euler_angles() {
    //         Ok(val) => {
    //             euler_angles = val;
    //             println!("IMU angles: {:?}", euler_angles);
    //             std::thread::sleep(std::time::Duration::from_millis(500));
    //         }
    //         Err(e) => {
    //             eprintln!("{:?}", e);
    //         }
    //     }
    // }
}
