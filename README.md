# Raspberry Pi Pico Minimal PWM interrupt handler

This is a minimal Rust program for the Raspberry Pi Pico that
sets up a PWM channel and enables an interrupt handler for it.
Unfortunately, when the interrupt handler is called, the entire
program crashes immediately and I don't know why.

This program sends debug output to UART0 on pins 1 and 2 of the Pico, at 115200
baud.  If you compile and run this program as-is, you should get output like
this:
```
Hello from pico-pwm-irq-crash
t0: 14794 t1: 14801
t0: 14794 t1: 16389
t0: 14794 t1: 18206
t0: 14794 t1: 20029
t0: 14794 t1: 21851
t0: 14794 t1: 23674
t0: 14794 t1: 25497
t0: 14794 t1: 27320
t0: 14794 t1: 29142
t0: 14794 t1: 30965
t0: 14794 t1: 32788
t0: 14794 t1: 34611
t0: 14794 t1: 36434
```
... and the onboard LED will blink at 1Hz.

If you locate these lines, and change the ``false`` to ``true``:
```
    // Change this to ``true`` to cause the crash:
    if false {
```
then you will get output like this:
```
Hello from pico-pwm-irq-crash
t0: 14795 t1: 14802
t0: 14795 t1: 16389
t0: 14795 t1: 18206
t0: 14795 t1: 20029
t0: 14795 t1: 21851
t0: 14795 t1: 23674
t0: 14795 t1: 25497
t0: 14795 t1: 27320
t0: 14795 t1: 29142
t0: 14795 t1: 30965
t0: 14795 t1: 32788
t
```
The output ends abruptly and incompletely, right around 20ms after the PWM was
enabled and the handler function should have been called. The LED will _not_
blink because the main program has crashed! This is the bug I'm trying to fix.
The PWM interrupt handler function is empty and should not cause problems.

-----

For general info on Rust and RP2040/Raspberry Pi Pico programming:
https://github.com/rp-rs/rp-hal

License for this repository: [MIT](https://opensource.org/licenses/MIT)
