use esp_idf_hal::i2c::*; // I2C peripheral
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::units::Hertz;
//use bno055::{BNO055OperationMode, Bno055};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::delay::Ets; // équivalent de Delay
// use mint::{EulerAngles, Quaternion}; //mathg tool to calculate angle
// use byteorder::{ByteOrder, LittleEndian};

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


    //write 0b01 in 0x3d to activate only accelerometer (look at BNO055OperationMode)
    let _ = i2c.write(0x28,&[0x3d, 0b0111], 100);
    
    FreeRtos::delay_ms(1000);

    let mut buffer = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    loop {
        let res = i2c.write_read(0x28, &[0x08], &mut buffer, 100);
        println!("Result: {res:?}");
        println!("Buffer: {buffer:?}");
        //this beautiful part dont let me see all the data so I comment it 
        // Accéléromètre
        let accel_x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let accel_y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let accel_z = i16::from_le_bytes([buffer[4], buffer[5]]);
        println!("Accel -> X: {accel_x}, Y: {accel_y}, Z: {accel_z}");
        // Magnetometer
        let mag_x = i16::from_le_bytes([buffer[6], buffer[7]]);
        let mag_y = i16::from_le_bytes([buffer[8], buffer[9]]);
        let mag_z = i16::from_le_bytes([buffer[10], buffer[11]]);
        println!("Mag -> X: {mag_x}, Y: {mag_y}, Z: {mag_z}");
        // Gyroscope
        let gyro_x = i16::from_le_bytes([buffer[12], buffer[13]]);
        let gyro_y = i16::from_le_bytes([buffer[14], buffer[15]]);
        let gyro_z = i16::from_le_bytes([buffer[16], buffer[17]]);
        //println!("Gyro -> X: {gyro_x}, Y: {gyro_y}, Z: {gyro_z}");
        FreeRtos::delay_ms(1000);
    }

}
