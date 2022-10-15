#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use microbit::{
    board::Board,
    display::blocking::{Display},
    hal::{
        //clocks::Clocks,
        //rtc::{Rtc, RtcInterrupt},
        twim,
        Timer,
    },
    pac::{
        //self,
        //interrupt,
        //RTC0,
        //TIMER1,
        twim0::frequency::FREQUENCY_A,
        //TWIM0,
    },
};

use lsm303agr::{
    AccelMode,
    AccelOutputDataRate,
    Lsm303agr
};

use defmt_rtt as _;
use panic_halt as _;

#[entry]
fn main() -> ! {
    let board = Board::take().unwrap();
    let mut timer = Timer::new(board.TIMER0);

    let i2c = { twim::Twim::new(board.TWIM0, board.i2c_internal.into(), FREQUENCY_A::K100) };

    let mut sensor = Lsm303agr::new_with_i2c(i2c);
    match sensor.accelerometer_id() {
        Ok(0x33u8) => {}
        _ => defmt::panic!("accelerometer not found"),
    }
    sensor.init().unwrap();
    sensor.set_accel_odr(AccelOutputDataRate::Hz100).unwrap();

    sensor.set_accel_mode(AccelMode::HighResolution).unwrap();

    let mut display = Display::new(board.display_pins);

    let left = [
        [1, 0, 0, 0, 0],
        [1, 0, 0, 0, 0],
        [1, 0, 0, 0, 0],
        [1, 0, 0, 0, 0],
        [1, 0, 0, 0, 0],
    ];

    let right = [
        [0, 0, 0, 0, 1],
        [0, 0, 0, 0, 1],
        [0, 0, 0, 0, 1],
        [0, 0, 0, 0, 1],
        [0, 0, 0, 0, 1],
    ];
    loop {
        let data = sensor.accel_data().unwrap();
        if data.x < 0 {
            display.show(&mut timer, left, 10);
        } else {
            display.show(&mut timer, right, 10)
        }
    }
}
