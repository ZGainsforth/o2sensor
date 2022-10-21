# o2sensor

This code expects the pico development environment to be set up (link?).

First clone the source.
```
git clone https://www.github.com/ZGainsforth/o2sensor
cd o2sensor
git submodule update --init --recursive
```

Next make the project.
```
mkdir build
cd build
cmake ..
make -j4
```

If you have a Raspberry Pi connected to the pico via SWD then you can make and push in one go:

``` make -j4 && openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program o2sensor.elf verify reset exit"

In addition to output from the oled, information is also reported through serial on pins 1,2,3 at 115200 baud 8N1.  It an be accessed for example via the following if hooked up to a RPi:

```minicom -b 115200 -o -D/dev/serial0
