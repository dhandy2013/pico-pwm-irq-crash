# Raspberry Pi Pico Minimal PWM interrupt handler

This is a minimal Rust program for the Raspberry Pi Pico that
sets up a PWM channel and enables an interrupt handler for it.
Unfortunately, when the interrupt handler is called, the entire
program crashes immediately and I don't know why.

For general info on Rust and RP2040/Raspberry Pi Pico programming:
https://github.com/rp-rs/rp-hal

License for this repository: [MIT](https://opensource.org/licenses/MIT)
