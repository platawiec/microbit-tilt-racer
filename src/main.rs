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
            rng,
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

    use cortex_m::interrupt::Mutex;
    use core::{cell::RefCell};

    use rand::{RngCore, SeedableRng};
    use rand_pcg::Pcg32;

    static RNG: Mutex<RefCell<Option<Pcg32>>> = Mutex::new(RefCell::new(None));


    use lsm303agr::{interface::I2cInterface, mode, AccelMode, AccelOutputDataRate, Lsm303agr};

    pub struct Game {
        player: Player,
        obstacle: Obstacle,
        track_speed: u8,
        score: u32,
        rng: rand_pcg::Lcg64Xsh32
    }

    struct Obstacle {
        board_position_x: usize,
        position_y: i32,
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
    const BASE_VELOCITY: u8 = 100;

    fn render_state(game: &Game) -> GreyscaleImage {
        // returns a GrayscaleImage from the game state
        let player = &game.player;
        let obstacle = &game.obstacle;
        let mut data = [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ];

        let board_position = get_board_position(player.position);
        let board_position_obstacle_y = get_board_position(obstacle.position_y);

        data[4][board_position as usize] = 9;
        data[board_position_obstacle_y][obstacle.board_position_x] = 5;
        GreyscaleImage::new(&data)
    }

    fn get_board_position(position: i32) -> usize {
        let track_delta = (TRACK_MAX - TRACK_MIN + 2 * TRACK_MARGIN) / 5;
        let pos = position - TRACK_MIN + TRACK_MARGIN;
        (pos / track_delta).clamp(0, 4) as usize
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

        /* Use hardware RNG to initialise PRNG */
        let mut rng = rng::Rng::new(board.RNG);
        let mut seed: [u8; 16] = [0; 16];
        let rng = Pcg32::from_seed(seed);
        
        let display = Display::new(board.TIMER1, board.display_pins);

        let mut player = Player {
            position: TRACK_MIN,
            velocity: 0,
            acceleration: 0,
        };
        let mut obstacle = Obstacle {
            board_position_x: 2,
            position_y: TRACK_MIN,
        };
        let mut game = Game {
            player,
            obstacle,
            track_speed: 1,
            score: 0,
            rng,
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
        let mut obstacle = &mut local.game.obstacle;

        local.game_clock.reset_event(RtcInterrupt::Tick);

        // check collisions

        // "spawn" new obstacle
        if obstacle.position_y > TRACK_MAX {
            let position_x = local.game.rng.next_u32() / (u32::MAX / 5);
            obstacle.board_position_x = position_x.clamp(0, 4) as usize;
            obstacle.position_y = TRACK_MIN;
        }
        
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

        obstacle.position_y += (BASE_VELOCITY as i32) * (local.game.track_speed as i32);
        
        shared.display.lock(|display| {
            display.show(&render_state(&local.game));
        });

        local.game.score += 1;

        // Speed up every minute
        if local.game.score % 3600 == 0 {
            local.game.track_speed += 1;
        }
    }
}
