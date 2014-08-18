apmled
======

This Teensy 3.1 Arduino sketch listens on Serial2 of the Teensy for MAVLink Heartbeat messages. It flashes, blinks, fades or just turns on solid lights based on the mode. The LEDs are single color LEDs with an Adafruit 12-Channel 16-bit PWM LED Driver driven in BitBanging SPI like mode.

- Disarmed: dim slow blinking
- Low Battery / other emergency: bright fast blinking
- RTL: fast flashing of FL&RR and FR&RL in sync
- Auto mission: all LEDs async flashing
- All others: front solid, rear async flashing
