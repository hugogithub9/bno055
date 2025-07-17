use esp_idf_hal::i2c::*; // I2C peripheral
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::units::Hertz;
//use bno055::{BNO055OperationMode, Bno055};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::delay::Ets; // équivalent de Delay
use mint::{EulerAngles, Quaternion}; //mathg tool to calculate angle
use byteorder::{ByteOrder, LittleEndian};

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");

    let peripherals = Peripherals::take().unwrap();
    let config = I2cConfig::new().baudrate(Hertz::from(100_000)); //max 400kHz
                                                                  //let config = I2cConfig::new().baudrate(100_i32.kHz().into());
    let sda = peripherals.pins.gpio6;
    let scl = peripherals.pins.gpio7;
    let mut i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).expect("error at initialisation of I2C driver");
    println!("Variables created!");

    //test
    // println!("Scanning I2C bus..."); //send an empty trame to every possible i2c adress
    // for addr in 0x00..0x29 {
    //     if i2c.write(addr, &[], 100).is_ok() {
    //         //if write dont have error, a device exist at this adress
    //         println!("Device found at 0x{:02X}", addr); //print if found a device
    //     }
    // }

    let mut delay = Ets {};
    //let mut imu = Bno055::new(i2c).with_alternative_address();
    println!("Others variables created!");
    FreeRtos::delay_ms(700); //wait for the sensor to be ready

    println!("Trying to initialize BNO055...");

//imu.init
    //set_page0
    let _ = i2c.write(0x28, &[0x07, 0], 100);
    //soft_reset
    let _ = i2c.write(0x28, &[0x3F, 0x20], 100);
    FreeRtos::delay_ms(650);
    //set_mode
    let _ = i2c.write(0x28,&[0x3d, 0b0], 100);
    FreeRtos::delay_ms(19);
    //set_power_mode
    let _ = i2c.write(0x28,&[0x3e, 0], 100);

    let _ = i2c.write(0x28,&[0x3f, 0], 100);


//imu.set_mode
    //set_page0
    //write 0b01 in 0x3d to activate only accelerometer (look at BNO055OperationMode)
    let _ = i2c.write(0x28,&[0x3d, 0b0111], 100);
    FreeRtos::delay_ms(19);

// read the value of the acceleration data ( 6 registers ÑSB and LSB for x, y, z)
    /*let mut buffer = [0,0,0,0,0,0];
    loop {
        let res = i2c.write_read(0x28, &[0x08], &mut buffer, 100);
        println!("Result: {res:?}");
        println!("Buffer: {buffer:?}");
        // Accéléromètre
        let accel_x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let accel_y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let accel_z = i16::from_le_bytes([buffer[4], buffer[5]]);
        println!("Accel -> X: {accel_x}, Y: {accel_y}, Z: {accel_z}");
        FreeRtos::delay_ms(1000);
   }*/

//read all the data of the sensor : acceleration, magnetude, gyroscope
    //write 0b0100 in 0x3d to activate the accelerometer and the magnetometer (look at BNO055OperationMode) line60
    //write 0b0111 in 0x3d to activate the accelerometer, magnetometer ans gyroscope (look at BNO055OperationMode) line60
    //let _ = i2c.write(0x28,&[0x3d, 0b0100], 100);
    //FreeRtos::delay_ms(19);

    let mut buffer = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    loop {
        let res = i2c.write_read(0x28, &[0x08], &mut buffer, 100);
        println!("Result: {res:?}");
        println!("Buffer: {buffer:?}");
        FreeRtos::delay_ms(1000);
        //this beautiful part dont let me see all the data so I comment it 
        /*// Accéléromètre
        let accel_x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let accel_y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let accel_z = i16::from_le_bytes([buffer[4], buffer[5]]);
        println!("Accel -> X: {accel_x}, Y: {accel_y}, Z: {accel_z}");
        // Magnetometer
        let mag_x = i16::from_le_bytes([buffer[6], buffer[7]]);
        let mag_y = i16::from_le_bytes([buffer[8], buffer[9]]);
        let mag_z = i16::from_le_bytes([buffer[10], buffer[11]]);
        println!("Mag -> X: {mag_x}, Y: {mag_y}, Z: {mag_z}");
        FreeRtos::delay_ms(1000);
        // Gyroscope
        let gyro_x = i16::from_le_bytes([buffer[12], buffer[13]]);
        let gyro_y = i16::from_le_bytes([buffer[14], buffer[15]]);
        let gyro_z = i16::from_le_bytes([buffer[16], buffer[17]]);
        println!("Gyro -> X: {gyro_x}, Y: {gyro_y}, Z: {gyro_z}");*/

    }

    let mut buffer = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    let res = i2c.write_read(0x28, &[0x08], &mut buffer, 100);
    println!("Result: {res:?}");
    println!("Buffer: {buffer:?}");
    FreeRtos::delay_ms(1000);
/* 
    // These are sensor fusion reading using the mint crate that the state will be read into
    let mut euler_angles: EulerAngles<f32, ()>; // = EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]);
    let mut quaternion: Quaternion<f32>; // = Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]);
    // Quaternion; due to a bug in the BNO055, this is recommended over Euler Angles

    //match imu.quaternion() { -> explain the intern fct
    //set page0 (already done)
    //if is fusion enabled : //const IMU = 0b1000;
                             //const COMPASS = 0b1001;
                             //const M4G = 0b1010;
                             //const NDOF_FMC_OFF = 0b1011;
                             //const NDOF = 0b1100;
    loop {
    let mut buf: [u8; 8] = [0; 8];

    //  I2C read of quaternion registers (0x20 à 0x27)
    match i2c.write_read(0x28, &[0x20], &mut buf, 100) {
        Ok(_) => {
            let w = LittleEndian::read_i16(&buf[0..2]);
            let x = LittleEndian::read_i16(&buf[2..4]);
            let y = LittleEndian::read_i16(&buf[4..6]);
            let z = LittleEndian::read_i16(&buf[6..8]);

            let scale = 1.0 / (1 << 14) as f32;

            let quat = mint::Quaternion {
                v: mint::Vector3 {
                    x: x as f32 * scale,
                    y: y as f32 * scale,
                    z: z as f32 * scale,
                },
                s: w as f32 * scale,
            };

            println!("IMU Quaternion: {:?}", quat);
        }
        Err(e) => {
            eprintln!("Erreur de lecture I2C : {:?}", e);
        }
    }

    std::thread::sleep(std::time::Duration::from_millis(500));
}*/


    //     // Euler angles, directly read
    //     match imu.euler_angles() {
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


/* 
    let mut buffer1 = [0,0,0,0,0,0];
    let mut buffer2 = [0,0,0,0,0,0];
    let mut buffer3 = [0,0,0,0,0,0];
    loop {
        let res1 = i2c.write_read(0x28, &[0x08], &mut buffer1, 100);
        let res2 = i2c.write_read(0x28, &[0x0E], &mut buffer2, 100);
        let res3 = i2c.write_read(0x28, &[0x14], &mut buffer3, 100);
        println!("Result: {res1:?}");
        println!("Buffer: {buffer1:?}");
        println!("Result: {res2:?}");
        println!("Buffer: {buffer2:?}");
        println!("Result: {res3:?}");
        println!("Buffer: {buffer3:?}");
        // Accéléromètre
        let accel_x = i16::from_le_bytes([buffer1[0], buffer1[1]]);
        let accel_y = i16::from_le_bytes([buffer1[2], buffer1[3]]);
        let accel_z = i16::from_le_bytes([buffer1[4], buffer1[5]]);
        println!("Accel -> X: {accel_x}, Y: {accel_y}, Z: {accel_z}");
        // Magnetometer
        let mag_x = i16::from_le_bytes([buffer2[0], buffer2[1]]);
        let mag_y = i16::from_le_bytes([buffer2[2], buffer2[3]]);
        let mag_z = i16::from_le_bytes([buffer2[4], buffer2[5]]);
        println!("Mag -> X: {mag_x}, Y: {mag_y}, Z: {mag_z}");
        FreeRtos::delay_ms(1000);
        // Gyroscope
        let gyro_x = i16::from_le_bytes([buffer3[0], buffer3[1]]);
        let gyro_y = i16::from_le_bytes([buffer3[2], buffer3[3]]);
        let gyro_z = i16::from_le_bytes([buffer3[4], buffer3[5]]);
        println!("Gyro -> X: {gyro_x}, Y: {gyro_y}, Z: {gyro_z}");
        FreeRtos::delay_ms(1000);
    }*/

}
