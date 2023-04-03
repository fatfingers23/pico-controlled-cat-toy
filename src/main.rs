//! # Pico W Blinky Example
//!
//! Blinks the LED on a Pico W board.
//!
//! This will blink an LED attached to WL_GPIO0, which is the pin the Pico W uses for
//! the on-board LED. It is connected to the the Wifi chip so it cannot be set using RP2040 pins
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]


mod pin_wrappers;

use cortex_m::delay::Delay;
use cyw43::NetDevice;
use rp_pico_w::entry;
use hal::gpio::dynpin::DynPin;

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use pin_wrappers::InOutPin;
use rp_pico_w::hal::{Clock, pac, Sio};

use rp_pico_w::gspi::GSpi;
use rp_pico_w::hal;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;

use embassy_executor::raw::TaskPool;
use embassy_executor::Executor;
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Timer};
use rp_pico_w::hal::multicore::Multicore;

/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static
/// values is reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything seperately
/// and modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte
/// alignment, which allows the stack guard to take up the least amount of
/// usable RAM.
static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();


/// The function configures the RP2040 peripherals, initializes
/// networking, then blinks the LED in an infinite loop.
/// TODO: add some simple network service
#[entry]
fn main() -> ! {
    info!("start");

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let _clocks = hal::clocks::init_clocks_and_plls(
        rp_pico_w::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    let mut sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico_w::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let sys_freq = _clocks.system_clock.freq().to_Hz();
    // Start up the second core to blink the second LED
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(sys_freq)
    });

    info!("init time driver");
    let timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);
    unsafe { rp_pico_w::embassy_time_driver::init(timer) };

    let mut executor = Executor::new();

    // Safety: function never returns, executor is never dropped
    let executor: &'static mut Executor = unsafe { forever_mut(&mut executor) };

    let task_pool: TaskPool<_, 1> = TaskPool::new();
    let task_pool = unsafe { forever(&task_pool) };

    let state = cyw43::State::new();
    let state = unsafe { forever(&state) };

    info!("run spawner");
    executor.run(|spawner| {
        let spawn_token = task_pool.spawn(|| run(spawner, pins, state));
        spawner.spawn(spawn_token).unwrap();
    });
}

///Core 1 runs the stepper motor and laser logic
fn core1_task(sys_freq: u32) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };

    let mut sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);

    // Pins for the stepper motor
    let in1 = InOutPin::new(pins.gpio18.into());
    let in2 = InOutPin::new(pins.gpio19.into());
    let in3 = InOutPin::new(pins.gpio20.into());
    let in4 = InOutPin::new(pins.gpio21.into());

    // Stepper motor sequence to turn clockwise
    let clockwise_sequence = [
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1],
    ];

    let counter_clockwise_sequence = [
        [0,0,0,1],
        [0,0,1,0],
        [0,1,0,0],
        [1,0,0,0],
    ];
    let mut motor_pins = [in1, in2, in3, in4];



    loop {
        // info!("core1_task:");
        run_the_motor(counter_clockwise_sequence, &mut motor_pins, &mut delay);

        // let input = sio.fifo.read();
        // if let Some(word) = input {
        //     delay.delay_ms(word);
        //     // led_pin.toggle().unwrap();
        //     // sio.fifo.write_blocking(CORE1_TASK_COMPLETE);
        // };
    }
}

/// Runs the motor with the given sequence
fn run_the_motor(sequence: [[u8; 4]; 4], motor_pins: &mut [InOutPin; 4], mut delay: &mut Delay) {
    for step in &sequence {
        for (i, &value) in step.iter().enumerate() {
            set_pin_value(&mut motor_pins[i], value);
            //5 Slow
            //2 Fast
            // Timer::after(Duration::from_millis(2)).await
            delay.delay_ms(2)
        }
    }
}

/// Sets the value of the given pin
fn set_pin_value(pin: &mut InOutPin, value: u8) {
    if value == 1 {
        pin.set_high().unwrap();
    } else {
        pin.set_low().unwrap();
    }
}

// TODO documentation
unsafe fn forever_mut<T>(r: &'_ mut T) -> &'static mut T {
    core::mem::transmute(r)
}

// TODO documentation
unsafe fn forever<T>(r: &'_ T) -> &'static T {
    core::mem::transmute(r)
}

async fn run(spawner: Spawner, pins: rp_pico_w::Pins, state: &'static cyw43::State) -> ! {
    // These are implicitly used by the spi driver if they are in the correct mode
    let mut spi_cs: hal::gpio::dynpin::DynPin = pins.wl_cs.into();
    // TODO should be high from the beginning :-(
    spi_cs.into_readable_output();
    spi_cs.set_high().unwrap();
    spi_cs.into_push_pull_output();
    spi_cs.set_high().unwrap();

    let mut spi_clk = pins.voltage_monitor_wl_clk.into_push_pull_output();
    spi_clk.set_low().unwrap();

    let mut spi_mosi_miso: hal::gpio::dynpin::DynPin = pins.wl_d.into();
    spi_mosi_miso.into_readable_output();
    spi_mosi_miso.set_low().unwrap();
    spi_mosi_miso.into_push_pull_output();
    spi_mosi_miso.set_low().unwrap();

    let bus = GSpi::new(spi_clk, spi_mosi_miso);
    let spi = embedded_hal_bus::spi::ExclusiveDevice::new(bus, spi_cs);

    let pwr = pins.wl_on.into_push_pull_output();

    let fw = cyw43::firmware::firmware();
    let clm = cyw43::firmware::clm();

    use embassy_futures::yield_now;
    yield_now().await;

    info!("create cyw43 driver");
    let (mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    let task_pool: TaskPool<_, 1> = TaskPool::new();
    let task_pool = unsafe { forever(&task_pool) };
    let spawn_token = task_pool.spawn(|| runner.run());
    spawner.spawn(spawn_token).unwrap();

    info!("init net net device");
    let net_device = control.init(clm).await;
    info!("init net net device done");

    if option_env!("WIFI_NETWORK").is_some() {
        if option_env!("WIFI_PASSWORD").is_some() {
            control
                .join_wpa2(
                    option_env!("WIFI_NETWORK").unwrap(),
                    option_env!("WIFI_PASSWORD").unwrap(),
                )
                .await;
        } else {
            control
                .join_open(option_env!("WIFI_NETWORK").unwrap())
                .await;
        }
    } else {
        warn!("Environment variable WIFI_NETWORK not set during compilation - not joining wireless network");
    }
    let config = embassy_net::ConfigStrategy::Dhcp;
    //let config = embassy_net::ConfigStrategy::Static(embassy_net::Config {
    //    address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 69, 2), 24),
    //    dns_servers: Vec::new(),
    //    gateway: Some(Ipv4Address::new(192, 168, 69, 1)),
    //});

    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

    let mut stack_resources = embassy_net::StackResources::<1, 2, 8>::new();
    let stack_resources = unsafe { forever_mut(&mut stack_resources) };

    // Init network stack
    let stack = Stack::new(net_device, config, stack_resources, seed);
    let stack = unsafe { forever(&stack) };

    let task_pool: TaskPool<_, 1> = TaskPool::new();
    let task_pool = unsafe { forever(&task_pool) };
    let spawn_token = task_pool.spawn(|| stack.run());
    let spawn_token_two = task_pool.spawn(|| loop{

    });
    spawner.spawn(spawn_token).unwrap();

    //Socket buffers
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];


    //Program loop running networking
    loop {

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_net::SmolDuration::from_secs(10)));

        info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            warn!("accept error: {:?}", e);
            continue;
        }

        info!("Received connection from {:?}", socket.remote_endpoint());

        loop {


            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    warn!("read error: {:?}", e);
                    break;
                }
            };

            info!("rxd {:02x}", &buf[..n]);
            // match socket.write() { }
            //
            match socket.write(&buf[..n]).await {
                Ok(_0) => {
                    info!("wrote");
                    "Hello World!"
                }
                Err(e) => {
                    warn!("write error: {:?}", e);
                    break;
                }
            };

        }


    }







}