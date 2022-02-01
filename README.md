# Raspberry Pi Pico Minimal PWM interrupt handler

This is a minimal Rust program for the Raspberry Pi Pico that sets up a PWM
channel and enables an interrupt handler for it.  This program demonstrates
that you *must* clear the PWM interrupt inside of the interrupt handler
function or else the program will crash.

This program sends debug output to UART0 on pins 1 and 2 of the Pico, at 115200
baud.  If you compile and run this program as-is, you should get output like
this:
```
Hello from pico-pwm-irq-crash
t0: 14826 t1: 14832
t0: 14826 t1: 16407
t0: 14826 t1: 18224
t0: 14826 t1: 20047
t0: 14826 t1: 21870
t0: 14826 t1: 23692
t0: 14826 t1: 25515
t0: 14826 t1: 27338
t0: 14826 t1: 29161
t0: 14826 t1: 30984
t0: 14826 t1: 32807
t0: 14826 t1: 34629
t0: 14826 t1: 36452
```
... and the onboard LED will blink at 1Hz.

If you locate the these lines in ``src/main.rs``:
```
            // If you comment-out this line, this program will crash:
            pwm.clear_interrupt();
```
and comment out the pwm.clear_interrupt() call, you will get output like this:
```
Hello from pico-pwm-irq-crash
t0: 14826 t1: 14832
t0: 14826 t1: 16407
t0: 14826 t1: 18224
t0: 14826 t1: 20047
t0: 14826 t1: 21870
t0: 14826 t1: 23692
t0: 14826 t1: 25515
t0: 14826 t1: 27338
t0: 14826 t1: 29161
t0: 14826 t1: 30984
t0: 14826 t1: 32807
t0
```
The output ends abruptly and incompletely, right around 20ms after the PWM was
enabled and the handler function should have been called. The LED will _not_
blink because the main program has crashed! This is because the PWM interrupt
did not get cleared in the PWM interrupt handler function, and therefore the
function will immediately be called again as soon as it exits -- forever.

-----

For general info on Rust and RP2040/Raspberry Pi Pico programming:
https://github.com/rp-rs/rp-hal

License for this repository: [MIT](https://opensource.org/licenses/MIT)
