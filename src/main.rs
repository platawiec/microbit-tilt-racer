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
            gpiote::Gpiote,
            rng,
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

    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

    use rand::{RngCore, SeedableRng};
    use rand_pcg::Pcg32;

    static RNG: Mutex<RefCell<Option<Pcg32>>> = Mutex::new(RefCell::new(None));

    use lsm303agr::{interface::I2cInterface, mode, AccelMode, AccelOutputDataRate, Lsm303agr};

    enum GameState {
        StartMenu,
        Play,
        Collision,
        ScoreMenu,
    }

    pub struct Game {
        player: Player,
        obstacle: Obstacle,
        track_speed: u8,
        score: u32,
        rng: rand_pcg::Lcg64Xsh32,
        state: GameState,
        anim_timer: u8,
        collision_animation: [[u8; 5]; 5],
    }
    impl Game {
        fn new(player: Player, obstacle: Obstacle, rng: rand_pcg::Lcg64Xsh32) -> Game {
            Game {
                player,
                obstacle,
                track_speed: 1,
                score: 0,
                rng,
                state: GameState::StartMenu,
                anim_timer: 0,
                collision_animation: [[0; 5]; 5],
            }
        }
    }

    struct Obstacle {
        board_position_x: usize,
        position_y: i32,
    }
    impl Obstacle {
        fn new(x: u8) -> Obstacle {
            Obstacle {
                board_position_x: x as usize,
                position_y: TRACK_MIN,
            }
        }
    }

    struct Player {
        position: i32,
        velocity: i32,
        acceleration: i32,
    }
    impl Player {
        fn new() -> Player {
            Player {
                position: 0,
                velocity: 0,
                acceleration: 0,
            }
        }
    }

    #[shared]
    struct Shared {
        display: Display<pac::TIMER1>,
        gpiote: Gpiote,
        game: Game,
    }

    #[local]
    struct Local {
        game_clock: Rtc<pac::RTC0>,
        sensor: Lsm303agr<I2cInterface<twim::Twim<TWIM0>>, mode::MagOneShot>,
    }

    const TRACK_MIN: i32 = -2000;
    const TRACK_MAX: i32 = 2000;
    const TRACK_MARGIN: i32 = 500; // track area for "gutters"
    const BASE_VELOCITY: u8 = 100;

    fn render_state(game: &Game) -> GreyscaleImage {
        // returns a GrayscaleImage from the game state
        let data = match game.state {
            GameState::StartMenu => [
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
                [0, 0, 9, 0, 0],
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
            ],
            GameState::Play => {
                let mut data = [[0u8; 5]; 5];
                let player = &game.player;
                let obstacle = &game.obstacle;

                let board_position = get_board_position(player.position);
                let board_position_obstacle_y = get_board_position(obstacle.position_y);

                data[4][board_position as usize] = 9;
                data[board_position_obstacle_y][obstacle.board_position_x] = 5;
                data
            }
            GameState::Collision => game.collision_animation,
            GameState::ScoreMenu => score_image(game.score, game.anim_timer),
        };

        GreyscaleImage::new(&data)
    }

    fn get_board_position(position: i32) -> usize {
        let track_delta = (TRACK_MAX - TRACK_MIN + 2 * TRACK_MARGIN) / 5;
        let pos = position - TRACK_MIN + TRACK_MARGIN;
        (pos / track_delta).clamp(0, 4) as usize
    }

    fn score_image(score: u32, anim_timer: u8) -> [[u8; 5]; 5] {
        let mut data = [[0u8; 5]; 5];
        let brightness = anim_timer / 6;
        let filled_col = (score / 3600).clamp(0, 4);
        let last_row = ((score % 3600) / (3600 / 5)).clamp(0, 4);
        for i in 0..=filled_col {
            if i < filled_col {
                for j in 0..5 {
                    data[j as usize][i as usize] = brightness;
                }
            } else {
                for j in 0..=last_row {
                    data[j as usize][i as usize] = brightness;
                }
            };
        }
        data
    }

    fn play<'a>(
        game: &'a mut Game,
        sensor: &mut Lsm303agr<I2cInterface<twim::Twim<TWIM0>>, mode::MagOneShot>,
    ) -> &'a mut Game {
        let mut player = &mut game.player;
        let mut obstacle = &mut game.obstacle;

        // check collisions
        if get_board_position(obstacle.position_y) == 4
            && (get_board_position(player.position) == obstacle.board_position_x)
        {
            game.state = GameState::Collision;
            game.collision_animation = [[0u8; 5]; 5];
            game.collision_animation[4][get_board_position(player.position)] = 9;
            game.anim_timer = 0;
        }

        // "spawn" new obstacle
        if obstacle.position_y > TRACK_MAX {
            let position_x = game.rng.next_u32() / (u32::MAX / 5);
            obstacle.board_position_x = position_x.clamp(0, 4) as usize;
            obstacle.position_y = TRACK_MIN;
        }

        let data = sensor.accel_data().unwrap();

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

        obstacle.position_y += (BASE_VELOCITY as i32) * (game.track_speed as i32);

        game.score += 1;

        // Speed up every minute
        if game.score % 3600 == 0 {
            game.track_speed += 1;
        };

        game
    }

    fn animate_collision<'a>(game: &'a mut Game) -> &'a mut Game {
        let mut rng = &mut game.rng;
        let radius: i32 = (game.anim_timer / 10) as i32;
        let position: i32 = get_board_position(game.player.position) as i32;
        // update rate: 30 fps
        if game.anim_timer % 2 == 0 {
            for i in 0usize..5usize {
                let rand_num = rng.next_u64();
                let rand_num = rand_num.to_ne_bytes();
                let rand_num = rand_num.map(|byte| (byte / (u8::MAX / 10)).clamp(0, 9));
                for j in 0usize..5usize {
                    let d = (j as i32 - position).abs() + (4 - i as i32).abs();
                    if d < radius {
                        game.collision_animation[i][j] = rand_num[j];
                    } else if d == radius {
                        game.collision_animation[i][j] = rand_num[j] / 2;
                    }
                }
            }
        };
        // end of animation, show score menu
        if game.anim_timer == 59 {
            game.state = GameState::ScoreMenu;
        };
        game
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

        let mut gpiote = Gpiote::new(board.GPIOTE);
        let channel0 = gpiote.channel0();
        channel0
            .input_pin(&board.buttons.button_a.degrade())
            .hi_to_lo()
            .enable_interrupt();
        channel0.reset_events();

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

        let mut player = Player::new();
        let mut obstacle = Obstacle::new(2);
        let mut game = Game::new(player, obstacle, rng);
        (
            Shared {
                display,
                gpiote,
                game,
            },
            Local {
                game_clock: rtc0,
                sensor,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = TIMER1, priority = 3, shared = [display])]
    fn timer1(mut cx: timer1::Context) {
        cx.shared
            .display
            .lock(|display| display.handle_display_event());
    }

    #[task(binds = GPIOTE, priority = 2, shared = [gpiote, game])]
    fn gpiote(mut cx: gpiote::Context) {
        let mut shared = cx.shared;

        let button_a_pressed = shared.gpiote.lock(|gpiote| {
            let a_pressed = gpiote.channel0().is_event_triggered();
            gpiote.channel0().reset_events();
            a_pressed
        });

        shared.game.lock(|game| {
            let current_state = &game.state;
            match (current_state, button_a_pressed) {
                (GameState::StartMenu, true) => game.state = GameState::Play,
                (GameState::ScoreMenu, true) => {
                    // reset the game
                    game.player = Player::new();
                    game.obstacle = Obstacle::new(2);
                    game.state = GameState::StartMenu;
                    game.score = 0;
                }
                _ => (),
            }
        });
    }

    #[task(binds = RTC0, priority = 1, shared = [display, game], local = [game_clock, sensor])]
    fn rtc0(cx: rtc0::Context) {
        let mut shared = cx.shared;
        let local = cx.local;

        local.game_clock.reset_event(RtcInterrupt::Tick);

        let image = shared.game.lock(|game| {
            let game = match game.state {
                GameState::StartMenu => game,
                GameState::Play => play(game, local.sensor),
                GameState::Collision => animate_collision(game),
                GameState::ScoreMenu => game,
            };
            game.anim_timer += 1;
            game.anim_timer %= 60;
            render_state(game)
        });

        shared.display.lock(|display| {
            display.show(&image);
        });
    }
}
