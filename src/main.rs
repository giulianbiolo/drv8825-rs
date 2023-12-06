use rppal::gpio::{Gpio, Error};
use spin_sleep::SpinSleeper;

mod drv8825;
use drv8825::{DRV8825, Direction};

// const DRV8825_ENABLE_PIN: u8 = 17; // BCM 17 = Physical 11, Enable Pin
const DRV8825_STEP_PIN: u8 = 27; // BCM 27 = Physical 13, Step Pin
const DRV8825_DIR_PIN: u8 = 22; // BCM 22 = Physical 15, Direction Pin
const MICROSTEPS: u8 = 1;
const RADIUS: f64 = 0.05;
const ACCEL: f64 = 0.01;

fn main() -> Result<(), Error> {
    let gpio: Gpio = Gpio::new()?;
    let spin_sleep: SpinSleeper = SpinSleeper::default();
    let mut stepper: DRV8825 = DRV8825::new(&gpio, None, DRV8825_STEP_PIN, DRV8825_DIR_PIN, MICROSTEPS, RADIUS, ACCEL)?;
    println!("Hello, world!");
    stepper.enable()?;
    stepper.set_direction(Direction::Forward)?;
    
    stepper.travel_ease_in(&spin_sleep, 2.5, 0.1);

    stepper.disable()?;
    Ok(())
}
