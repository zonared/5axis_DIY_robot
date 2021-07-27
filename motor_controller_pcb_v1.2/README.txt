This folder contains the files for the PCB I attempted to design as a bi-directional PWM motor controller with options for other projects (prototyping area)

The motor controller board consisted of the following items and features:
  A PIC microprocessor 16F88 which has built-in RS232 UART, PWM and external interrupts to name a few
  RS232 TTL line driver (MAX232)
  LM324 op-amp for amplifying hall effect signals (this was before I figured out you can buy digital output hall effects)
  74LS08 AND gate for h-bridge (this was before I figured you could buy dedicated h-bridge output pairs built into microprocessors)
  MPC17511AEP Bipolar Motor Driver CMOS Parallel 24-QFN-EP (aka h-bridge, since replaced with DRV8833 or POLOLU-2130)
  LM7805 linear regulator (good, but these are better OKI-78SR-5/1.5-W36-C by Murata Power)

It didn't actually work out that great, had lots of electrical noise issues and daisychaining RS232 is a bad idea, this is why I changed to i2c board to board comms.

I have actually used some of these boards on other projects, so it wasn’t a total waste.

