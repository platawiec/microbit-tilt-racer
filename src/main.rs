#![deny(unsafe_code)]
#![no_main]
#![no_std]


use defmt_rtt as _;
use panic_halt as _;

use rtic::app;

#[app(device = microbit::pac, peripherals = true)]
mod app{
    use microbit::{
        board::Board,
        display::nonblocking::{Display, GreyscaleImage},
        hal::{
            clocks::Clocks,
            rtc::{Rtc, RtcInterrupt},
            twim,
            Timer,
        },
        pac::{
            self,
            //interrupt,
            //RTC0,
            //TIMER1,
            twim0::frequency::FREQUENCY_A,
            TWIM0,
        },
    };

    use lsm303agr::{
        AccelMode,
        AccelOutputDataRate,
        Lsm303agr,
        interface::I2cInterface,
        mode,
    };

    #[shared]
    struct Shared {
        display: Display<pac::TIMER1>,
    }

    #[local]
    struct Local {
        game_clock: Rtc<pac::RTC0>,
        sensor: Lsm303agr<I2cInterface<twim::Twim<TWIM0>>, mode::MagOneShot>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let board = Board::new(cx.device, cx.core);
        
        // Start the low-frequency clock for RTC
        Clocks::new(board.CLOCK).start_lfclk();

        // RTC at ~60Hz (32_768 / (545+1))
        let mut rtc0 = Rtc::new(board.RTC0, 545).unwrap();
        rtc0.enable_event(RtcInterrupt::Tick);
        rtc0.enable_interrupt(RtcInterrupt::Tick, None);
        rtc0.enable_counter();

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

        let display = Display::new(board.TIMER1, board.display_pins);

        (
            Shared {
                display,
            },
            Local {
                game_clock: rtc0,
                sensor: sensor,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = TIMER1, priority = 2, shared = [display])]
    fn timer1(mut cx: timer1::Context) {
        cx.shared
            .display
            .lock(|display| display.handle_display_event());
    }

    #[task(binds = RTC0, priority = 1, shared = [display], local = [game_clock, sensor, step: u8 = 0])]
    fn rtc0(cx: rtc0::Context) {
        let mut shared = cx.shared;
        let local = cx.local;

        local.game_clock.reset_event(RtcInterrupt::Tick);

        let data = local.sensor.accel_data().unwrap();
        let this_frame = if data.x < 0 {
            GreyscaleImage::new(&[
                [8, 0, 0, 0, 0],
                [8, 0, 0, 0, 0],
                [8, 0, 0, 0, 0],
                [8, 0, 0, 0, 0],
                [8, 0, 0, 0, 0],
            ])
        } else {
            GreyscaleImage::new(&[
                [0, 0, 0, 0, 8],
                [0, 0, 0, 0, 8],
                [0, 0, 0, 0, 8],
                [0, 0, 0, 0, 8],
                [0, 0, 0, 0, 8],
            ])
        };

        shared.display.lock(|display| {
            display.show(&this_frame);
        });

    }

}