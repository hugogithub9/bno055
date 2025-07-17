use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::units::Hertz;

// Fonction qui écrit et lit le mode actif
fn set_and_check_mode(i2c: &mut I2cDriver<'_>, mode: u8) {
    println!("Setting mode 0b{:04b}...", mode);

    // Étape 1 : mettre le capteur en CONFIG_MODE avant tout
    let res1 = i2c.write(0x28, &[0x3D, 0b0000], 100);
    FreeRtos::delay_ms(25);
    if let Err(e) = res1 {
        println!("Erreur pour passer en CONFIG_MODE: {:?}", e);
        return;
    }

    // Étape 2 : écrire le mode voulu
    let res2 = i2c.write(0x28, &[0x3D, mode], 100);
    FreeRtos::delay_ms(20);
    if let Err(e) = res2 {
        println!("Erreur pour écrire le mode 0x3D: {:?}", e);
        return;
    }

    // Étape 3 : lire le registre 0x3D pour vérifier
    let mut response = [0u8];
    let res3 = i2c.write_read(0x28, &[0x3D], &mut response, 100);
    if let Err(e) = res3 {
        println!("Erreur lors de la lecture du registre 0x3D: {:?}", e);
        return;
    }

    println!("Registre 0x3D = 0b{:04b} (décimal {})", response[0], response[0]);
    match response[0] {
        0b0000 => println!("Mode: CONFIG_MODE"),
        0b0001 => println!("Mode: ACC_ONLY"),
        0b0010 => println!("Mode: MAG_ONLY"),
        0b0011 => println!("Mode: GYRO_ONLY"),
        0b0100 => println!("Mode: ACC_MAG"),
        0b0101 => println!("Mode: ACC_GYRO"),
        0b0110 => println!("Mode: MAG_GYRO"),
        0b0111 => println!("Mode: AMG ✅"),
        0b1000 => println!("Mode: IMU"),
        0b1001 => println!("Mode: COMPASS"),
        0b1010 => println!("Mode: M4G"),
        0b1011 => println!("Mode: NDOF_FMC_OFF"),
        0b1100 => println!("Mode: NDOF"),
        _ => println!("Mode inconnu ou invalide"),
    }
}

fn main() {
    // Initialisation des périphériques
    let peripherals = Peripherals::take().unwrap();

    // GPIO SDA et SCL (adapter selon ton branchement)
    let sda = peripherals.pins.gpio4;  // Exemple : GPIO 4
    let scl = peripherals.pins.gpio5;  // Exemple : GPIO 5

    // Créer l'instance I2C avec config par défaut (400kHz ici)
    let i2c_config = I2cConfig::new().baudrate(Hertz::from(100_000));
    //let i2c_config = I2cConfig::new().baudrate(400.kHz().into());
    let mut i2c = I2cDriver::new(
        peripherals.i2c0, // ou i2c1 selon ta carte
        sda,
        scl,
        &i2c_config,
    ).unwrap();

    // Délai pour laisser le capteur booter
    FreeRtos::delay_ms(650); // recommandé pour le BNO055

    // Appel de la fonction qui écrit/lit le mode
    set_and_check_mode(&mut i2c, 0b0111); // AMG
}