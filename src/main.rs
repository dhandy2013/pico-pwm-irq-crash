//! Raspberry Pi Pico program: Show PWM interrupt handler crash
//! Pins and signals used:
//! - GPIO0: Pin 1: UART tx -> USB-serial rx
//! - GPIO1: Pin 2: UART rx -> USB-serial tx
//! - GND  : Pin 3: Ground  -> USB-serial ground
//! - GP25: Onboard LED
#![no_std]
#![no_main]

use core::fmt::Write; // for write!(), writeln!()
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::*;
use panic_halt as _;
use rp_pico::hal::{self, pac::interrupt, prelude::*};

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let core = hal::pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4 for 50.0288 Hz when fsys is 125MHz
    let pwm = &mut pwm_slices.pwm4;
    pwm.default_config();
    pwm.set_div_int(38);
    pwm.set_div_frac(2);
    pwm.set_top(65535);

    // Single-cycle I/O block
    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // UART for debug printing
    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_115200_8_N_1,
            clocks.peripheral_clock.into(),
        )
        .unwrap();
    writeln!(uart, "Hello from pico-pwm-irq-crash\r").unwrap();

    // Useful microsecond-resolution timer
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    // LED for diagnostic signals
    let mut led = pins.led.into_push_pull_output();

    // Running this block of code enables the PWM_IRQ_WRAP interrupt handler
    // function to be called. Unfortunately, whenever PWM_IRQ_WRAP() is called
    // the main program crashes immediately. Even if the function is empty.
    if false {
        // Tell the NVIC to start calling the PWM interrupt handler function
        unsafe {
            hal::pac::NVIC::unmask(hal::pac::Interrupt::PWM_IRQ_WRAP);
        }
    }

    // Enable the interrupt for this PWM slice
    pwm.enable_interrupt();
    // Start the PWM clock running
    pwm.enable();

    let t0 = timer.get_counter();
    let mut t1 = t0;

    // Wait till 20 milliseconds have passed, to give time for the PWM interrupt
    // handler to be called.
    while (t1 - t0) < 20_000 {
        t1 = timer.get_counter();
        writeln!(uart, "t0: {} t1: {}\r", t0, t1).unwrap();
    }

    // Slow blink to signal end of program (if we get this far!)
    loop {
        led.set_high().unwrap();
        delay.delay_ms(500);
        led.set_low().unwrap();
        delay.delay_ms(500);
    }
}

/// PWM interrupt handler function. Notice it is empty and should be harmless.
#[interrupt]
fn PWM_IRQ_WRAP() {}
