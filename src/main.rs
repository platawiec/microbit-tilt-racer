#![deny(unsafe_code)]
#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_halt as _;

use rtic::app;

#[app(device = microbit::pac, peripherals = true)]
mod app {
    use microbit::{
        board::Board,
        display::nonblocking::{Display, GreyscaleImage},
        hal::{
            clocks::Clocks,
            rtc::{Rtc, RtcInterrupt},
            twim, Timer,
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

    use lsm303agr::{interface::I2cInterface, mode, AccelMode, AccelOutputDataRate, Lsm303agr};

    pub struct Game {
        player: Player,
        track_speed: u8,
        score: u32,
    }

    struct Player {
        position: i32,
        velocity: i32,
        acceleration: i32,
    }

    #[shared]
    struct Shared {
        display: Display<pac::TIMER1>,
    }

    #[local]
    struct Local {
        game_clock: Rtc<pac::RTC0>,
        sensor: Lsm303agr<I2cInterface<twim::Twim<TWIM0>>, mode::MagOneShot>,
        game: Game,
    }

    const TRACK_MIN: i32 = -2000;
    const TRACK_MAX: i32 = 2000;
    const TRACK_MARGIN: i32 = 500; // track area for "gutters"

    fn render_state(player: &Player) -> GreyscaleImage {
        // returns a GrayscaleImage from the game state
        let mut data = [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ];

        let board_position = player_board_position(player);

        data[4][board_position as usize] = 9;
        GreyscaleImage::new(&data)
    }

    fn player_board_position(player: &Player) -> u8 {
        let track_delta = (TRACK_MAX - TRACK_MIN + 2 * TRACK_MARGIN) / 5;
        let pos = player.position - TRACK_MIN + TRACK_MARGIN;
        (pos / track_delta).clamp(0, 4) as u8
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

        sensor.set_accel_mode(AccelMode::Normal).unwrap();

        let display = Display::new(board.TIMER1, board.display_pins);

        let mut player = Player {
            position: 0,
            velocity: 0,
            acceleration: 0,
        };
        let mut game = Game {
            player,
            track_speed: 1,
            score: 0,
        };
        (
            Shared { display },
            Local {
                game_clock: rtc0,
                sensor,
                game
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

    #[task(binds = RTC0, priority = 1, shared = [display], local = [game_clock, sensor, game])]
    fn rtc0(cx: rtc0::Context) {
        let mut shared = cx.shared;
        let local = cx.local;
        let mut player = &mut local.game.player;

        local.game_clock.reset_event(RtcInterrupt::Tick);

        let data = local.sensor.accel_data().unwrap();

        // 1 game tick = 1/60 sec
        player.acceleration = data.x / 60;
        player.velocity += player.acceleration;
        if player.position == TRACK_MIN && player.velocity < 0 {
            player.velocity = 0;
        } else if player.position == TRACK_MAX && player.velocity > 0 {
            player.velocity = 0;
        }
        player.velocity = player.velocity.clamp(-100, 100);
        player.position += player.velocity;
        player.position = player.position.clamp(TRACK_MIN, TRACK_MAX);
        
        shared.display.lock(|display| {
            display.show(&render_state(&player));
        });

        local.game.score += 1;
    }
}
